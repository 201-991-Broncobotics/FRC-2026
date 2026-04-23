// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.auton.Autos;
import frc.robot.commands.StartIntakingCommand;
import frc.robot.commands.DisableLeftLimelightCommand;
import frc.robot.commands.DisableRightLimelightCommand;
import frc.robot.commands.DropIntakeCommand;
import frc.robot.commands.EnableTurretCommand;
import frc.robot.commands.LaunchBallsCommand;
import frc.robot.commands.IntakeAndLaunchBalls;
import frc.robot.commands.LiftIntakeCommand;
import frc.robot.commands.ResetElevatorCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ZoneConstants;
import frc.robot.Settings.RobotSettings;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DrivingProfiles;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TraverseSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.utility.LimelightHelpers;
import frc.robot.utility.OverrideController;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static edu.wpi.first.units.Units.*;

import java.util.Optional;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    public static double MaxAngularRate = RotationsPerSecond.of(1.0).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                           // second max angular
                                                                                           // velocity
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.02)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.FieldCentric povControl = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driver = new CommandXboxController(OperatorConstants.driverControllerPort);
    private final CommandXboxController operator = new CommandXboxController(OperatorConstants.operatorControllerPort);
    private final XboxController driverHID = driver.getHID();
    private final XboxController operatorHID = operator.getHID();


    private final OverrideController override = new OverrideController(5, driver, operator, 0.05);

    // The robot's subsystems and commands are defined here...
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(drivetrain);
    private final TraverseSubsystem traverseSubsystem = new TraverseSubsystem();
    private final OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem(drivetrain, operator);
    //private final ClimbingSubsystem climbingSubsystem = new ClimbingSubsystem();
    private final DrivingProfiles drivingProfile = new DrivingProfiles(drivetrain);

    private final SendableChooser<Command> autoChooser;

    private final StartIntakingCommand startIntakingCommand = new StartIntakingCommand(intakeSubsystem, traverseSubsystem);
    private final DropIntakeCommand dropIntakeCommand = new DropIntakeCommand(intakeSubsystem);
    private final LiftIntakeCommand liftIntakeCommand = new LiftIntakeCommand(intakeSubsystem);
    private final EnableTurretCommand enableTurretCommand = new EnableTurretCommand(outtakeSubsystem);
    //private final ResetElevatorCommand resetElevatorCommand = new ResetElevatorCommand(climbingSubsystem);
    private final LaunchBallsCommand launchBallsCommand = new LaunchBallsCommand(intakeSubsystem, traverseSubsystem);
    private final IntakeAndLaunchBalls IntakeAndLaunchBallsCommand = new IntakeAndLaunchBalls(intakeSubsystem, traverseSubsystem);
    private final DisableRightLimelightCommand disableRLimelightCommand = new DisableRightLimelightCommand();
    private final DisableLeftLimelightCommand disableLLimelightCommand = new DisableLeftLimelightCommand();

    private UsbCamera driverView, driverView2;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        driverView = CameraServer.startAutomaticCapture(0); // for usb camera, most likely on port 10.9.91.11
        driverView.setExposureAuto();
        driverView.setWhiteBalanceAuto();//setWhiteBalanceManual(0);
        driverView.setResolution(160, 120); //320, 240 // 16:9 aspect ratio for amcrest web cams
        driverView.setPixelFormat(PixelFormat.kMJPEG);
        driverView.setFPS(15);

        driverView2 = CameraServer.startAutomaticCapture(1); // for usb camera, most likely on port 10.9.91.11
        driverView2.setExposureAuto();
        driverView2.setWhiteBalanceAuto();
        driverView2.setResolution(160, 120); //320, 240
        driverView2.setPixelFormat(PixelFormat.kMJPEG);
        driverView2.setFPS(15);
        // "http://limelight.local:5800/stream.mjpg");
        
        NamedCommands.registerCommand("DropIntake", dropIntakeCommand);
        NamedCommands.registerCommand("LiftIntake", liftIntakeCommand);
        NamedCommands.registerCommand("StartIntaking", startIntakingCommand);
        NamedCommands.registerCommand("LaunchBalls", launchBallsCommand);
        NamedCommands.registerCommand("IntakeAndLaunchBalls", IntakeAndLaunchBallsCommand);
        NamedCommands.registerCommand("StopIntaking", new ParallelCommandGroup(
            new InstantCommand(intakeSubsystem::stopRollers, intakeSubsystem), 
            new InstantCommand(traverseSubsystem::stopRoller, traverseSubsystem))); // just by requiring these subsystems it should run the end part of the intaking command
        NamedCommands.registerCommand("EnableTurret", enableTurretCommand);
        NamedCommands.registerCommand("DisableTurret", new InstantCommand(outtakeSubsystem::stopShooting, outtakeSubsystem));
        NamedCommands.registerCommand("DisableRightLimelight", disableRLimelightCommand);
        NamedCommands.registerCommand("DisableLeftLimelight", disableLLimelightCommand);
        

        drivetrain.configureAutoBuilder(); // THIS HAS TO GO AFTER NAMED COMMANDS OMG

        autoChooser = AutoBuilder.buildAutoChooser("Example Path");
        SmartDashboard.putData("Auto Mode:", autoChooser);

        if(RobotSettings.overrideMode){
            outtakeSubsystem.setController(override);
        }

        

        configureBindings();

        /*driverView.setExposureHoldCurrent();
        driverView.setWhiteBalanceHoldCurrent();
        driverView2.setExposureHoldCurrent();
        driverView2.setWhiteBalanceHoldCurrent();*/
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {

        if(!RobotSettings.overrideMode){
            //Override Version 
            //DRIVE CONTROLS
            drivingProfile.setUpControllerInputs(
                () -> -driverHID.getLeftY(), // forward/backward
                () -> driverHID.getLeftX(), // strafe
                () -> -driverHID.getRightX(), // rotation
                () -> 0.35 + 0.65 * driverHID.getRightTriggerAxis(), // driving throttle
                () -> 0.5 + 0.8 * driverHID.getRightTriggerAxis(), // rotation throttle
                2, 3 // applies quadratic/cubic/quartic/etc. normalization to the drive inputs
            );

            // drivingProfile.getControllers(driver, operator);

            drivingProfile.setDefaultCommand(new RunCommand(drivingProfile::update, drivingProfile));
            drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() ->
                    drive.withVelocityX(drivingProfile.getForwardOutput() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-drivingProfile.getStrafeOutput() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(drivingProfile.getRotationOutput() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                )
            ); 

            // Robot Centric while holding back pedal (needs to be programed on the controller to the dpad up button)
            override.povUp().onTrue(new InstantCommand(drivingProfile::enableRobotCentric))
                .onFalse(new InstantCommand(drivingProfile::disableRobotCentric));

            //Brake (b)
            override.b().whileTrue(drivetrain.applyRequest(() -> brake));
            
            //Reset Heading (Y)
            override.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()))
                .onTrue(drivetrain.runOnce(() -> drivetrain.seedLL4Imu()));

            //Auto Align For Trenches
            override.povDown().onTrue(new InstantCommand(drivingProfile::enableAutoAlign))
                .onFalse(new InstantCommand(drivingProfile::disableAutoAlign));

            //Intake
            override.rightBumper().and(override.leftBumper().negate()) // just intake
                .toggleOnTrue(new InstantCommand(intakeSubsystem::feed))
                .toggleOnTrue(new InstantCommand(traverseSubsystem::transfer));

            //Reverse Intake
            override.rightBumper().and(override.leftBumper())
                .toggleOnTrue(new InstantCommand(intakeSubsystem::reverseFeed))
                .toggleOnTrue(new InstantCommand(traverseSubsystem::emergencyReverse));

            // Stop Intake
            override.rightBumper().negate()
                .toggleOnTrue(new InstantCommand(intakeSubsystem::stopRollers))
                .toggleOnTrue(new InstantCommand(traverseSubsystem::stopRoller));

            //Pivot (POV Left + Right)
            override.povLeft().toggleOnTrue(new InstantCommand(intakeSubsystem::lift));
            override.povRight().toggleOnTrue(new InstantCommand(intakeSubsystem::drop));

            //Flywheel (A)
            override.a().toggleOnTrue(new InstantCommand(outtakeSubsystem::toggleShooting));

            override.back().toggleOnTrue(new InstantCommand(intakeSubsystem::slightlyRaise)).toggleOnFalse(new InstantCommand(intakeSubsystem::drop));

            //Climb (POV Up + Down (driver only if fly speed is not automated))
            //override.povUp().whileTrue(new InstantCommand(climbingSubsystem::justExtend, climbingSubsystem)).toggleOnFalse(new InstantCommand(climbingSubsystem::stop, climbingSubsystem)); 
            //override.povDown().whileTrue(new InstantCommand(climbingSubsystem::justRetract, climbingSubsystem)).toggleOnFalse(new InstantCommand(climbingSubsystem::stop, climbingSubsystem));

            override.x().toggleOnTrue(new InstantCommand(outtakeSubsystem::toggleDumbShooter));
            
            //Scoop (RS)
            override.leftTrigger(0.8).or(override.rightBumper()).and(override.leftBumper().negate()) // either shoot or intake
                .toggleOnTrue(new InstantCommand(intakeSubsystem::feed))
                .toggleOnTrue(new InstantCommand(traverseSubsystem::transfer));
            override.rightBumper().and(override.leftBumper()) //  intake but with reverse
                .toggleOnTrue(new InstantCommand(intakeSubsystem::reverseFeed))
                .toggleOnTrue(new InstantCommand(traverseSubsystem::emergencyReverse));
            override.leftTrigger(0.8).and(override.leftBumper()) // shoot but with reverse
                .toggleOnTrue(new InstantCommand(traverseSubsystem::emergencyReverseScoop));
            override.leftTrigger(0.8).and(override.rightBumper().negate()).and(override.leftBumper().negate()) // if just shoot
                .toggleOnTrue(new InstantCommand(traverseSubsystem::scoop))
                .toggleOnTrue(new InstantCommand(intakeSubsystem::agitate));
                //.toggleOnTrue(new InstantCommand(intakeSubsystem::slightlyRaise));
            override.rightBumper() // if intake is pressed at all
                .toggleOnTrue(new InstantCommand(intakeSubsystem::stopAgitate))
                .toggleOnTrue(new InstantCommand(intakeSubsystem::drop));
            override.leftTrigger(0.8).negate().and(override.rightBumper().negate()) // nothing
                .toggleOnTrue(new InstantCommand(traverseSubsystem::stopScoop))
                .toggleOnTrue(new InstantCommand(intakeSubsystem::stopRollers))
                .toggleOnTrue(new InstantCommand(traverseSubsystem::stopRoller))
                .toggleOnTrue(new InstantCommand(intakeSubsystem::stopAgitate));

        } else {
            //Override Version 
            //DRIVE CONTROLS
            drivingProfile.setUpControllerInputs(
                () -> -override.getLeftY(), // forward/backward
                () -> override.getLeftX(), // strafe
                () -> -override.getRightX(), // rotation
                () -> 0.35 + 0.65 * override.getRightTriggerAxis(), // driving throttle
                () -> 0.5 + 0.8 * override.getRightTriggerAxis(), // rotation throttle
                2, 3 // applies quadratic/cubic/quartic/etc. normalization to the drive inputs
            );

            drivingProfile.getControllers(driver, operator);

            drivingProfile.setDefaultCommand(new RunCommand(drivingProfile::update, drivingProfile));
            drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() ->
                    drive.withVelocityX(drivingProfile.getForwardOutput() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-drivingProfile.getStrafeOutput() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(drivingProfile.getRotationOutput() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                )
            ); 

            // Robot Centric while holding back pedal (needs to be programed on the controller to the dpad up button)
            override.povUp().onTrue(new InstantCommand(drivingProfile::enableRobotCentric))
                .onFalse(new InstantCommand(drivingProfile::disableRobotCentric));

            //Brake (b)
            override.b().whileTrue(drivetrain.applyRequest(() -> brake));
            
            //Reset Heading (Y)
            override.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()))
                .onTrue(drivetrain.runOnce(() -> drivetrain.seedLL4Imu()));

            //Auto Align For Trenches
            override.povDown().onTrue(new InstantCommand(drivingProfile::enableAutoAlign))
                .onFalse(new InstantCommand(drivingProfile::disableAutoAlign));

            //Intake
            override.rightBumper().and(override.leftBumper().negate()) // just intake
                .toggleOnTrue(new InstantCommand(intakeSubsystem::feed))
                .toggleOnTrue(new InstantCommand(traverseSubsystem::transfer));

            //Reverse Intake
            override.rightBumper().and(override.leftBumper())
                .toggleOnTrue(new InstantCommand(intakeSubsystem::reverseFeed))
                .toggleOnTrue(new InstantCommand(traverseSubsystem::emergencyReverse));

            // Stop Intake
            override.rightBumper().negate()
                .toggleOnTrue(new InstantCommand(intakeSubsystem::stopRollers))
                .toggleOnTrue(new InstantCommand(traverseSubsystem::stopRoller));

            //Pivot (POV Left + Right)
            override.povLeft().toggleOnTrue(new InstantCommand(intakeSubsystem::lift));
            override.povRight().toggleOnTrue(new InstantCommand(intakeSubsystem::drop));

            //Flywheel (A)
            override.a().toggleOnTrue(new InstantCommand(outtakeSubsystem::toggleShooting));

            override.back().toggleOnTrue(new InstantCommand(intakeSubsystem::slightlyRaise)).toggleOnFalse(new InstantCommand(intakeSubsystem::drop));

            //Climb (POV Up + Down (driver only if fly speed is not automated))
            //override.povUp().whileTrue(new InstantCommand(climbingSubsystem::justExtend, climbingSubsystem)).toggleOnFalse(new InstantCommand(climbingSubsystem::stop, climbingSubsystem)); 
            //override.povDown().whileTrue(new InstantCommand(climbingSubsystem::justRetract, climbingSubsystem)).toggleOnFalse(new InstantCommand(climbingSubsystem::stop, climbingSubsystem));

            override.x().toggleOnTrue(new InstantCommand(outtakeSubsystem::toggleDumbShooter));
            
            //Scoop (RS)
            override.leftTrigger(0.8).or(override.rightBumper()).and(override.leftBumper().negate()) // either shoot or intake
                .toggleOnTrue(new InstantCommand(intakeSubsystem::feed))
                .toggleOnTrue(new InstantCommand(traverseSubsystem::transfer));
            override.rightBumper().and(override.leftBumper()) //  intake but with reverse
                .toggleOnTrue(new InstantCommand(intakeSubsystem::reverseFeed))
                .toggleOnTrue(new InstantCommand(traverseSubsystem::emergencyReverse));
            override.leftTrigger(0.8).and(override.leftBumper()) // shoot but with reverse
                .toggleOnTrue(new InstantCommand(traverseSubsystem::emergencyReverseScoop));
            override.leftTrigger(0.8).and(override.rightBumper().negate()).and(override.leftBumper().negate()) // if just shoot
                .toggleOnTrue(new InstantCommand(traverseSubsystem::scoop))
                .toggleOnTrue(new InstantCommand(intakeSubsystem::agitate));
                //.toggleOnTrue(new InstantCommand(intakeSubsystem::slightlyRaise));
            override.rightBumper() // if intake is pressed at all
                .toggleOnTrue(new InstantCommand(intakeSubsystem::stopAgitate))
                .toggleOnTrue(new InstantCommand(intakeSubsystem::drop));
            override.leftTrigger(0.8).negate().and(override.rightBumper().negate()) // nothing
                .toggleOnTrue(new InstantCommand(traverseSubsystem::stopScoop))
                .toggleOnTrue(new InstantCommand(intakeSubsystem::stopRollers))
                .toggleOnTrue(new InstantCommand(traverseSubsystem::stopRoller))
                .toggleOnTrue(new InstantCommand(intakeSubsystem::stopAgitate));

        }

        outtakeSubsystem.setDefaultCommand(new InstantCommand(outtakeSubsystem::update, outtakeSubsystem));
        intakeSubsystem.setDefaultCommand(new InstantCommand(intakeSubsystem::update, intakeSubsystem));
        traverseSubsystem.setDefaultCommand(new InstantCommand(traverseSubsystem::update, traverseSubsystem));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

    }

    public void start() {// Stops the crashing
        Optional<Alliance> alliance = DriverStation.getAlliance();
        ZoneConstants.allianceZone
                .setZone((alliance.get() == Alliance.Red) ? ZoneConstants.redZone : ZoneConstants.blueZone);
        ZoneConstants.allianceHub = alliance.get() == Alliance.Red ? ZoneConstants.redHub : ZoneConstants.blueHub;
        ZoneConstants.alliance = alliance.get() == Alliance.Blue;

        if (alliance.get() == Alliance.Red)
            drivetrain.setOperatorPerspectiveForward(new Rotation2d(Math.toRadians(180)));
        else
            drivetrain.setOperatorPerspectiveForward(new Rotation2d(Math.toRadians(0)));

        //CommandScheduler.getInstance().schedule(resetElevatorCommand);
        
    }

    public void whileStopped() {
        drivetrain.defaultLL4ImuMode();
        drivingProfile.stopRumble();
    }

    public void whenStarted() {
        // drivetrain.seedLL4Imu();
        drivetrain.runningLL4ImuMode();
        intakeSubsystem.resetPivotPosition();
    }

    public void autoStarted() { drivingProfile.autoWasJustRun(); }
    public void teleOpStarted() { drivingProfile.teleOpWasJustRun(); }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return autoChooser.getSelected(); // Autos.exampleAuto(m_exampleSubsystem);
    }
}
