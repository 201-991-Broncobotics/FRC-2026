package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Settings;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.utility.ElapsedTime;
import frc.robot.utility.Functions;
import frc.robot.utility.LimelightHelpers;
import frc.robot.utility.LimelightHelpers.PoseEstimate;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

     /** Swerve request to apply during robot-centric path following */
     private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private double lastAngVelX = 0, lastAngVelY = 0, lastAngVelZ = 0;
    private ElapsedTime gyroTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private ArrayList<Double> limelightAX = new ArrayList<Double>(), limelightAY = new ArrayList<Double>();
    private ArrayList<Double> limelightBX = new ArrayList<Double>(), limelightBY = new ArrayList<Double>();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }
    
    public void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> {

                    ChassisSpeeds field = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getState().Pose.getRotation());
                    //SmartDashboard.putNumber("AUTODriving Field VX", field.vxMetersPerSecond);
                    //SmartDashboard.putNumber("AUTODriving Field VY", field.vyMetersPerSecond);
                    
                    setControl(m_pathApplyRobotSpeeds.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                );},
                Settings.PathFollowerController,
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    public Command followPathCommand(String pathName) {
        try{
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

            return AutoBuilder.pathfindThenFollowPath(path, Settings.FollowerConstraints); 

            
            // Non pathfinding manual path following


            /* 
            var config = RobotConfig.fromGUISettings();
            return new FollowPathCommand(
                    path,
                    () -> getState().Pose, // Robot pose supplier
                    () -> getState().Speeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                    ), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds, AND feedforwards
                    Settings.PathFollowerController,
                    config, // The robot configuration
                    () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                    },
                    this // Reference to this subsystem to set requirements
            ); */

        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    } // PathPlannerPath

    public Command followPathCommand(PathPlannerPath path) {
        return AutoBuilder.pathfindThenFollowPath(
                path,
                Settings.FollowerConstraints);
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        // Also update pigeon acceleration and rotation 
        gyroData.accelX = getPigeon2().getAccelerationX().getValueAsDouble() * 9.80665;
        gyroData.accelY = getPigeon2().getAccelerationY().getValueAsDouble() * 9.80665;
        gyroData.accelZ = getPigeon2().getAccelerationZ().getValueAsDouble() * 9.80665;

        gyroData.pitch = Math.toRadians(getPigeon2().getPitch().getValueAsDouble());
        gyroData.roll = Math.toRadians(getPigeon2().getRoll().getValueAsDouble());
        gyroData.yaw = Math.toRadians(getPigeon2().getYaw().getValueAsDouble());

        gyroData.angVelX = Math.toRadians(getPigeon2().getAngularVelocityXDevice().getValueAsDouble());
        gyroData.angVelY = Math.toRadians(getPigeon2().getAngularVelocityYDevice().getValueAsDouble());
        gyroData.angVelZ = Math.toRadians(getPigeon2().getAngularVelocityZDevice().getValueAsDouble());

        double FrameTime = gyroTimer.time();
        gyroTimer.reset();
        gyroData.angAccelX = (lastAngVelX - gyroData.angVelX) * FrameTime;
        gyroData.angAccelY = (lastAngVelY - gyroData.angVelY) * FrameTime;
        gyroData.angAccelZ = (lastAngVelZ - gyroData.angVelZ) * FrameTime;

        LimelightHelpers.SetRobotOrientation("limelight-a", getState().Pose.getRotation().getDegrees(), Math.toRadians(getState().Speeds.omegaRadiansPerSecond), 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation("limelight-b", getState().Pose.getRotation().getDegrees(), Math.toRadians(getState().Speeds.omegaRadiansPerSecond), 0, 0, 0, 0);


        try {
            if (Settings.useRLimelight) {
                // right side limelight: 0.361803m up, -0.050800m forward, 0.355600m right
                PoseEstimate limelightPoseEstimateA = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-a");
                if (limelightPoseEstimateA != null) {
                    SmartDashboard.putString("Right Limelight Pose:", Functions.stringifyPose(limelightPoseEstimateA.pose));
                    
                    if (LimelightHelpers.validPoseEstimate(limelightPoseEstimateA)) { 
                        double distVarA = 0.00329999 * Math.pow(limelightPoseEstimateA.avgTagDist, 2.2061);
                        double sideVarA = 0.00177127 * Math.pow(limelightPoseEstimateA.avgTagDist, 3.50671);
                        double roughTagAngle = Math.toRadians(getState().Pose.getRotation().getDegrees() - 90);
                        double xVarA = Math.abs(Math.cos(roughTagAngle)) * distVarA + Math.abs(Math.sin(roughTagAngle)) * sideVarA;
                        double yVarA = Math.abs(Math.sin(roughTagAngle)) * distVarA + Math.abs(Math.cos(roughTagAngle)) * sideVarA;

                        SmartDashboard.putNumber("LLA avgDist", limelightPoseEstimateA.avgTagDist);
                        SmartDashboard.putNumber("LLA xVar", xVarA);
                        SmartDashboard.putNumber("LLA yVar", yVarA);
                        
                        addVisionMeasurement(limelightPoseEstimateA.pose, limelightPoseEstimateA.timestampSeconds, VecBuilder.fill(xVarA, yVarA, Math.toRadians(10.0)));
                    } else {
                        limelightAX.add(0.0);
                        limelightAY.add(0.0);
                    }
                }
            }

            if (Settings.useLLimelight) {
                // left side limelight: 0.332161m up, 0.063500m forward, -0.355600m right
                PoseEstimate limelightPoseEstimateB = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-b");
                if (limelightPoseEstimateB != null) {
                    SmartDashboard.putString("Left Limelight Pose:", Functions.stringifyPose(limelightPoseEstimateB.pose));
                    
                    if (LimelightHelpers.validPoseEstimate(limelightPoseEstimateB)) {
                        limelightBX.add(limelightPoseEstimateB.pose.getX());
                        limelightBY.add(limelightPoseEstimateB.pose.getY());
                        double distVarB = 0.00329999 * Math.pow(limelightPoseEstimateB.avgTagDist, 2.2061);
                        double sideVarB = 0.00177127 * Math.pow(limelightPoseEstimateB.avgTagDist, 3.50671);
                        double roughTagAngle = Math.toRadians(getState().Pose.getRotation().getDegrees() + 90);
                        double xVarB = Math.abs(Math.cos(roughTagAngle)) * distVarB + Math.abs(Math.sin(roughTagAngle)) * sideVarB;
                        double yVarB = Math.abs(Math.sin(roughTagAngle)) * distVarB + Math.abs(Math.cos(roughTagAngle)) * sideVarB;

                        SmartDashboard.putNumber("LLB avgDist", limelightPoseEstimateB.avgTagDist);
                        SmartDashboard.putNumber("LLB xVar", xVarB);
                        SmartDashboard.putNumber("LLB yVar", yVarB);
                        
                        addVisionMeasurement(limelightPoseEstimateB.pose, limelightPoseEstimateB.timestampSeconds, VecBuilder.fill(xVarB, yVarB, Math.toRadians(10.0)));
                    } else {
                        limelightBX.add(0.0);
                        limelightBY.add(0.0);
                    }
                }
            }

            if (limelightAX.size() > Settings.stddevFrames) limelightAX.remove(0);
            if (limelightAY.size() > Settings.stddevFrames) limelightAY.remove(0);
            if (limelightBX.size() > Settings.stddevFrames) limelightBX.remove(0);
            if (limelightBY.size() > Settings.stddevFrames) limelightBY.remove(0);

            SmartDashboard.putNumber("LLA stddev X", Functions.standardDeviation(limelightAX, 5));
            SmartDashboard.putNumber("LLA stddev Y", Functions.standardDeviation(limelightAY, 5));
            SmartDashboard.putNumber("LLB stddev X", Functions.standardDeviation(limelightBX, 5));
            SmartDashboard.putNumber("LLB stddev Y", Functions.standardDeviation(limelightBY, 5));
            
        } catch (NullPointerException e) {
            // do nothing
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }


    public void setDriveMotorCurrentLimit(double amps) {
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit = amps;
        currentLimits.StatorCurrentLimitEnable = true; // default
        currentLimits.StatorCurrentLimit = 120; // default
        for (SwerveModule<TalonFX, TalonFX, CANcoder> module : getModules()) {
            module.getDriveMotor().getConfigurator().apply(currentLimits);
        }
    }

    public Translation3d getFieldCentricAcceleration() {
        // pigeon: +x is right, +y is down, +z is forward for current robot
        //robot relative is x is forward, y is left, and z is up
        Translation3d robotRelative = new Translation3d(gyroData.accelZ, -gyroData.accelX, -gyroData.accelY);
        return robotRelative.rotateBy(new Rotation3d(getState().Pose.getRotation()));
    }

    public Translation3d getFieldCentricVelocity() {
        Translation3d robotRelative = new Translation3d(getState().Speeds.vxMetersPerSecond, getState().Speeds.vyMetersPerSecond, 0);
        return robotRelative.rotateBy(new Rotation3d(getState().Pose.getRotation()));
    }

    public Rotation2d getAngAcceleration() {
        return new Rotation2d(-gyroData.angAccelY);
    }


    public static class gyroData { // accelerations are in Gs 
        public static double accelX = 0;
        public static double accelY = 0;
        public static double accelZ = 0;
        public static double pitch = 0;
        public static double roll = 0;
        public static double yaw = 0;
        public static double angVelX = 0;
        public static double angVelY = 0;
        public static double angVelZ = 0;
        public static double angAccelX = 0;
        public static double angAccelY = 0;
        public static double angAccelZ = 0;
    }
}
