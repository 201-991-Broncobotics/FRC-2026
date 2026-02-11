// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.auton.Autos;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DrivingProfiles;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static edu.wpi.first.units.Units.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(1.0).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.02)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.FieldCentric povControl = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


    // The robot's subsystems and commands are defined here...
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final StorageSubsystem storageSubsystem = new StorageSubsystem();
    private final OuttakeSubsystem turretSubsystem = new OuttakeSubsystem(); 
    private final ClimbingSubsystem climbingSubsystem = new ClimbingSubsystem(); 
    private final DrivingProfiles drivingProfile = new DrivingProfiles(drivetrain);

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driver =
        new CommandXboxController(OperatorConstants.driverControllerPort);
    private final CommandXboxController operator = 
        new CommandXboxController(OperatorConstants.operatorControllerPort); 

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the trigger bindings

        // CameraServer.startAutomaticCapture("Limelight", "http://limelight.local:5800/stream.mjpg");

        configureBindings();
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

        //DRIVER CONTROLS
        drivingProfile.setUpControllerInputs(
            () -> -driver.getLeftY(), // + ((driverJoystick.povUp().getAsBoolean())? 0.15:0.0) + ((driverJoystick.povDown().getAsBoolean())? -0.15:0.0), 
            () -> driver.getLeftX(), // + ((driverJoystick.povRight().getAsBoolean())? 0.15:0.0) + ((driverJoystick.povLeft().getAsBoolean())? -0.15:0.0), 
            () -> -driver.getRightX(), 
            () -> 0.3 + 0.7 * driver.getRightTriggerAxis(), 
            2, 2
        );

        drivingProfile.setDefaultCommand(new RunCommand(drivingProfile::update, drivingProfile));
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(drivingProfile.getForwardOutput() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-drivingProfile.getStrafeOutput() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(drivingProfile.getRotationOutput() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        ); 

        driver.b().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); // resets heading (but gets overridden by limelight)
        driver.a().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));




        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        new Trigger(m_exampleSubsystem::exampleCondition)
            .onTrue(new ExampleCommand(m_exampleSubsystem));


        //OPERATOR CONTROLS 
        operator.leftTrigger(0.05).toggleOnTrue(new InstantCommand(storageSubsystem::transfer, storageSubsystem)); 
        operator.leftTrigger(0.05).toggleOnFalse(new InstantCommand(storageSubsystem::stop, storageSubsystem));
        operator.leftBumper().toggleOnTrue(new InstantCommand(intakeSubsystem::lift, intakeSubsystem));
        operator.leftBumper().toggleOnFalse(new InstantCommand(intakeSubsystem::drop, intakeSubsystem));
        operator.povUp().whileTrue(new InstantCommand(climbingSubsystem::extend, climbingSubsystem)); 
        operator.povDown().whileTrue(new InstantCommand(climbingSubsystem::retract, climbingSubsystem)); 
        operator.rightTrigger(0.05).whileTrue(new InstantCommand(turretSubsystem::tuneFlywheel, turretSubsystem)); 
        operator.rightBumper().toggleOnTrue(new InstantCommand(intakeSubsystem::feed, intakeSubsystem)); 
        operator.rightBumper().toggleOnFalse(new InstantCommand(intakeSubsystem::stopRollers, intakeSubsystem));





        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return Autos.exampleAuto(m_exampleSubsystem);
    }
}
