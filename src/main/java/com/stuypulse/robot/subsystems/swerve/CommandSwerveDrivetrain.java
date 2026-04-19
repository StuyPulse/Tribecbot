/** ********************** PROJECT TRIBECBOT ************************ */
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
 /* Use of this source code is governed by an MIT-style license */
 /* that can be found in the repository LICENSE file.           */
/** ************************************************************ */
package com.stuypulse.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.RobotMode;
import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import com.stuypulse.robot.subsystems.superstructure.turret.Turret;
import com.stuypulse.robot.subsystems.swerve.TunerConstants.TunerSwerveDrivetrain;
import com.stuypulse.robot.util.PhoenixUtil;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Vector2D;

import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

    private final static CommandSwerveDrivetrain instance;

    private FieldObject2d turret2d = Field.FIELD2D.getObject("Turret 2D");
    private Pose2d turretPose = new Pose2d();
    // private StructPublisher<Pose2d> leftBehindHubYPlublisher;
    // private StructPublisher<Pose2d> rightBehindHubYPlublisher;
    // private StructPublisher<Pose2d> vertexBehindHubPublisher;
    private StatusSignal<LinearAcceleration> robotAccelerationX;
    private StatusSignal<LinearAcceleration> robotAccelerationY;

    private StatusSignal<Current>[] driveMotorSupplyCurrents;
    private StatusSignal<Current>[] steerMotorSupplyCurrents;

    //TODO: might wanna change some of these initialized values like isBehindTower indicates that we are in the alliance zone 
    private Optional<Boolean> isBehindHub = Optional.empty();
    private Optional<Boolean> isOutsideAllianceZone = Optional.empty();
    private Optional<Boolean> isInOpponentZone = Optional.empty();
    private Optional<Boolean> isUnderTrench = Optional.empty();
    private Optional<Boolean> isBehindTower = Optional.empty();

    // private StructPublisher<Pose2d> robotPose = NetworkTableInstance.getDefault()
    //         .getStructTopic("Robot Pose", Pose2d.struct).publish();

    static {
        instance = TunerConstants.createDrivetrain();
        // instance.registerTelemetry(instance::telemeterize);
    }

    // public void telemeterize(SwerveDriveState state) {
    // 	/* Write drive state to the log file */
    // 	SignalLogger.writeStruct("DriveState/Pose", Pose2d.struct, state.Pose);
    // 	SignalLogger.writeStruct("DriveState/Speeds", ChassisSpeeds.struct, state.Speeds);
    // 	SignalLogger.writeStructArray("DriveState/ModuleStates", SwerveModuleState.struct, state.ModuleStates);
    // 	SignalLogger.writeStructArray("DriveState/ModuleTargets", SwerveModuleState.struct, state.ModuleTargets);
    // 	SignalLogger.writeStructArray("DriveState/ModulePositions", SwerveModulePosition.struct, state.ModulePositions);
    // 	SignalLogger.writeStruct("DriveState/RawHeading", Rotation2d.struct, state.RawHeading);
    // 	SignalLogger.writeDouble("DriveState/Timestamp", state.Timestamp, "seconds");
    // 	SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");
    // 	SignalLogger.writeInteger("DriveState/FailedDaqs", state.FailedDaqs);
    // }
    public static CommandSwerveDrivetrain getInstance() {
        return instance;
    }

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private Pose2d lastGoodPose = new Pose2d(1.5, 1.5, Rotation2d.kZero);

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_moduleTranslationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withDeadband(Settings.Swerve.MODULE_VELOCITY_DEADBAND_M_PER_S)
            .withRotationalDeadband(Settings.Swerve.ROTATIONAL_DEADBAND_RAD_PER_S)
            .withDesaturateWheelSpeeds(true);

    private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withDeadband(Settings.Swerve.MODULE_VELOCITY_DEADBAND_M_PER_S)
            .withRotationalDeadband(Settings.Swerve.ROTATIONAL_DEADBAND_RAD_PER_S)
            .withDesaturateWheelSpeeds(true);

    public SwerveRequest.FieldCentric getFieldCentricSwerveRequest() {
        return this.fieldCentricRequest;
    }

    public SwerveRequest.RobotCentric getRobotCentricSwerveRequest() {
        return this.robotCentricRequest;
    }

    /*
	 * SysId routine for characterizing module translation. This is used to find PID
	 * gains for the drive motors.
     */
    private final SysIdRoutine m_sysIdRoutineModuleTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdModuleTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_moduleTranslationCharacterization.withVolts(output)),
                    null,
                    this));

    /*
	 * SysId routine for characterizing chassis translation. This is used to find
	 * PID gains PID to pose.
     */
    private final SysIdRoutine m_sysIdRoutineChassisTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in meters per second², but SysId only supports "volts per second" */
                    Volts.of(0.5).per(Second),
                    /* This is in meters per second, but SysId only supports "volts" */
                    Volts.of(Settings.Swerve.Constraints.MAX_VELOCITY_M_PER_S),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdChassisTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually meters per second, but SysId only supports "volts" */
                        setControl(getRobotCentricSwerveRequest().withVelocityX(output.in(Volts)).withVelocityY(0)
                                .withRotationalRate(0));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Target X Velocity ('voltage')", output.in(Volts));
                        SignalLogger.writeDouble("X Position", getPose().getX());
                        SignalLogger.writeDouble("X Velocity",
                                getChassisSpeeds().vxMetersPerSecond * getPose().getRotation().getCos());
                    },
                    null,
                    this));

    /*
	 * SysId routine for characterizing steer. This is used to find PID gains for
	 * the steer motors.
     */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    /*
	 * SysId routine for characterizing rotation.
	 * This is used to find PID gains for the FieldCentricFacingAngle
	 * HeadingController.
	 * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
	 * importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Target_Rate ('voltage')", output.in(Volts));
                        SignalLogger.writeDouble("Rotational Position", getPose().getRotation().getRadians());
                        SignalLogger.writeDouble("Rotational_Velocity", getState().Speeds.omegaRadiansPerSecond);
                    },
                    null,
                    this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineModuleTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct the devices themselves. If they need the devices, they can
     * access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules Constants for each specific module
     */
    protected CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        // leftBehindHubYPlublisher = NetworkTableInstance.getDefault().getStructTopic("FieldPositions/LeftBehindHubY", Pose2d.struct).publish();
        // rightBehindHubYPlublisher = NetworkTableInstance.getDefault().getStructTopic("FieldPositions/RightBehindHubY", Pose2d.struct).publish();
        // vertexBehindHubPublisher = NetworkTableInstance.getDefault().getStructTopic("FieldPositions/VertexBehindHub", Pose2d.struct).publish();

        robotAccelerationX = this.getPigeon2().getAccelerationX();
        robotAccelerationY = this.getPigeon2().getAccelerationY();

        driveMotorSupplyCurrents = new StatusSignal[4];
        steerMotorSupplyCurrents = new StatusSignal[4];

        for (int i = 0; i < 4; i++) {
            driveMotorSupplyCurrents[i] = getModule(i).getDriveMotor().getSupplyCurrent();
            steerMotorSupplyCurrents[i] = getModule(i).getSteerMotor().getSupplyCurrent();
        }

        PhoenixUtil.registerToCanivore(
                robotAccelerationX,
                robotAccelerationY
        );

        PhoenixUtil.registerToCanivore(driveMotorSupplyCurrents);
        PhoenixUtil.registerToCanivore(steerMotorSupplyCurrents);
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct the devices themselves. If they need the devices, they can
     * access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     * unspecified or set to 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN
     * 2.0.
     * @param modules Constants for each specific module
     */
    private CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct the devices themselves. If they need the devices, they can
     * access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     * unspecified or set to 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN
     * 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry
     * calculation in the form [x, y, theta]ᵀ, with units in meters and radians
     * @param visionStandardDeviation The standard deviation for vision
     * calculation in the form [x, y, theta]ᵀ, with units in meters and radians
     * @param modules Constants for each specific module
     */
    private CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    @Override
    public void setControl(SwerveRequest request) {
        if (EnabledSubsystems.SWERVE.get()) {
            super.setControl(request);
        } else {
            super.setControl(new SwerveRequest.Idle());
        }
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

    public Command sysidRotationDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineRotation.dynamic(direction);
    }

    public Command sysidRotationQuasiStatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineRotation.quasistatic(direction);
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

    private boolean checkIfVisionMeasurementValid(Pose2d visionPose) {
        return !Settings.ENABLE_DISTANCE_CHECK.get() || visionPose.getTranslation().getDistance(getState().Pose.getTranslation()) < Settings.Swerve.MAX_ACCEPTABLE_VISION_DEVIATION_METERS;
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the
     * vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in
     * seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        // SignalLogger.writeStruct("Vision/Pose", Pose2d.struct, visionRobotPoseMeters);
        // SignalLogger.writeDouble("Vision/Timestamp", timestampSeconds);

        if (checkIfVisionMeasurementValid(visionRobotPoseMeters)) {
            super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
        }
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this
     * method will continue to apply to future measurements until a subsequent
     * call to {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the
     * vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in
     * seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     * measurement in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        // SignalLogger.writeStruct("Vision/Pose", Pose2d.struct, visionRobotPoseMeters);
        // SignalLogger.writeDouble("Vision/Timestamp", timestampSeconds);

        if (checkIfVisionMeasurementValid(visionRobotPoseMeters)) {
            super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                    visionMeasurementStdDevs);
        }
    }

    public Pose2d getPose() {
        Pose2d proposedPose = getState().Pose;
        double proposedX = proposedPose.getX();
        double proposedY = proposedPose.getY();
        double poseDelta = lastGoodPose.getTranslation().getDistance(proposedPose.getTranslation());

        //  if (!(proposedX > Field.LENGTH || proposedX < 0 || proposedY > Field.WIDTH || proposedY < 0) &&
        // poseDelta <= Settings.Swerve.MAX_ACCEPTABLE_POSE_DELTA_METERS) {
        lastGoodPose = proposedPose;
        //  }

        return lastGoodPose;
    }

    public void configureAutoBuilder() {
        try {
            AutoBuilder.configure(
                    this::getPose,
                    this::resetPose,
                    this::getChassisSpeeds,
                    this::setChassisSpeeds,
                    new PPHolonomicDriveController(Gains.Swerve.Alignment.XY, Gains.Swerve.Alignment.THETA),
                    RobotConfig.fromGUISettings(),
                    () -> false,
                    instance);
            PathPlannerLogging.setLogActivePathCallback((poses) -> {
                if (Robot.isBlue()) {
                    Field.FIELD2D.getObject("path").setPoses(poses);

                } else {
                    Field.FIELD2D.getObject("path").setPoses(Field.transformToOppositeAlliance(poses));
                }
            });
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public Command followPathCommand(String pathName) {
        try {
            return followPathCommand(PathPlannerPath.fromPathFile(pathName));
        } catch (Exception e) {
            throw new IllegalArgumentException(pathName + " does not exist");
        }
    }

    public Command followPathCommand(PathPlannerPath path) {
        return AutoBuilder.followPath(path);
    }

    public SwerveModuleState[] getModuleStates() {
        return getState().ModuleStates;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return getState().Speeds;
    }

    public Vector2D getFieldRelativeSpeeds() {
        return new Vector2D(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond)
                .rotate(Angle.fromRotation2d(getPose().getRotation()));
    }

    private void setChassisSpeeds(ChassisSpeeds robotSpeeds) {
        setControl(new SwerveRequest.RobotCentric()
                .withVelocityX(robotSpeeds.vxMetersPerSecond)
                .withVelocityY(robotSpeeds.vyMetersPerSecond)
                .withRotationalRate(robotSpeeds.omegaRadiansPerSecond));
    }

    public void drive(Vector2D velocity, double rotation) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                Robot.isBlue() ? velocity.y : -velocity.y,
                Robot.isBlue() ? -velocity.x : velocity.x,
                -rotation,
                getPose().getRotation());

        Pose2d robotVel = new Pose2d(
                Settings.DT * speeds.vxMetersPerSecond,
                Settings.DT * speeds.vyMetersPerSecond,
                Rotation2d.fromRadians(Settings.DT * speeds.omegaRadiansPerSecond));
        Twist2d twistVel = new Pose2d().log(robotVel);

        setChassisSpeeds(new ChassisSpeeds(
                twistVel.dx / Settings.DT,
                twistVel.dy / Settings.DT,
                twistVel.dtheta / Settings.DT));
    }

    public Pose2d getTurretPose() {
        return turretPose;
    }

    public double[] getWheelDrivePositionsRadians() {
        double[] positions = new double[4];
        double kDriveGearRatio = 7.67;
        for (int i = 0; i < 4; i++) {
            positions[i] = getModule(i).getDriveMotor().getPosition().getValueAsDouble() * 2 * Math.PI
                    / kDriveGearRatio;
        }
        return positions;
    }

    public boolean isUnderTrench() {
        if (isUnderTrench.isEmpty()) {
            Translation2d turretTranslation = getTurretPose().getTranslation();

            boolean isBetweenRightTrenchesY = Field.AllianceRightTrench.rightEdge.getY() < turretTranslation.getY()
                    && Field.AllianceRightTrench.leftEdge.getY() > turretTranslation.getY();

            boolean isBetweenLeftTrenchesY = Field.AllianceLeftTrench.rightEdge.getY() < turretTranslation.getY()
                    && Field.AllianceLeftTrench.leftEdge.getY() > turretTranslation.getY();

            boolean isUnderAllianceTrenchX = Math.abs(
                    turretTranslation.getX() - Field.AllianceRightTrench.rightEdge.getX()) < Field.TRENCH_HOOD_TOLERANCE;

            boolean isUnderOpponentTrenchX = Math.abs(
                    turretTranslation.getX() - Field.OpponentRightTrench.rightEdge.getX()) < Field.TRENCH_HOOD_TOLERANCE;

            boolean isUnderTrench = (isBetweenRightTrenchesY || isBetweenLeftTrenchesY)
                    && (isUnderAllianceTrenchX || isUnderOpponentTrenchX);

            this.isUnderTrench = Optional.of(isUnderTrench);
        }

        return isUnderTrench.get();
    }

    public boolean isInOpponentZone() {
        if (isInOpponentZone.isEmpty()) {
            Translation2d turretTranslation = getTurretPose().getTranslation();
            isInOpponentZone = Optional.of(turretTranslation.getX() > Field.OPPONENT_ZONE_X);
        }
        return isInOpponentZone.get();
    }

    public boolean isBehindTower() {
        if (isBehindTower.isEmpty()) {
            boolean withinTowerX = getPose().getTranslation().getX() < Field.TOWER_FAR_CENTER.getX();
            boolean withinTowerY = Field.TOWER_FAR_RIGHT.getY() < getTurretPose().getTranslation().getY()
                    && getTurretPose().getTranslation().getY() < Field.TOWER_FAR_LEFT.getY();
            isBehindTower = Optional.of(withinTowerX && withinTowerY);
        }

        return isBehindTower.get();
    }

    /**
     * Checks whether the robot turret pose is behind the hub or not in the
     * neutral zone. For stopping ferrying to prevent fuel hitting the hub.
     *
     * @return true if robot turret pose is behind the hub.
     */
    public boolean isBehindHub() {
        if (isBehindHub.isEmpty()) {
            // === TRIANGLE === (CUSTOM VERTEX)
            Translation2d turretTranslation = getTurretPose().getTranslation();

            boolean behindHubX = Field.HUB_FAR_LEFT_CORNER.getX() < turretTranslation.getX();
            // && turretTranslation.getX() < Field.hubFarLeftCorner.getX() + Field.hubToleranceX; // With this line the triangle will be cut to more like a trapezoid.

            Pose2d hubFarLeftCornerWithTolerance = new Pose2d(Field.HUB_FAR_LEFT_CORNER.getX(), Field.HUB_FAR_LEFT_CORNER.getY() + Field.BEHIND_HUB_TOLERANCE_Y, new Rotation2d());
            Pose2d hubFarRightCornerWithTolerance = new Pose2d(Field.HUB_FAR_RIGHT_CORNER.getX(), Field.HUB_FAR_RIGHT_CORNER.getY() - Field.BEHIND_HUB_TOLERANCE_Y, new Rotation2d());

            // Find point on triangle using the point-slope formula (of the line constructed by the hub corner pose and ferry pose)
            // y = (slope)(robotX - hubCornerX) + (hubCornerY)
            // where the slope = (hubCornerY - vertexY)/(hubCornerX - vertexX)
            double leftY = ((hubFarLeftCornerWithTolerance.getY() - Field.BEHIND_HUB_TRIANGLE_VERTEX.getY())
                    / (hubFarLeftCornerWithTolerance.getX() - Field.BEHIND_HUB_TRIANGLE_VERTEX.getX())) // (Slope)
                    * (turretTranslation.getX() - hubFarLeftCornerWithTolerance.getX()) + hubFarLeftCornerWithTolerance.getY(); // *(robotX - hubCornerX) + (hubCornerY)
            double rightY = ((hubFarRightCornerWithTolerance.getY() - Field.BEHIND_HUB_TRIANGLE_VERTEX.getY())
                    / (hubFarRightCornerWithTolerance.getX() - Field.BEHIND_HUB_TRIANGLE_VERTEX.getX())) // (Slope)
                    * (turretTranslation.getX() - hubFarRightCornerWithTolerance.getX()) + hubFarRightCornerWithTolerance.getY(); // *(robotX - hubCornerX) + (hubCornerY)

            // Debug:
            // leftBehindHubYPlublisher.set(new Pose2d(getTurretPose().getX(), leftY, new Rotation2d()));
            // rightBehindHubYPlublisher.set(new Pose2d(getTurretPose().getX(), rightY, new Rotation2d()));
            // vertexBehindHubPublisher.set(Field.BEHIND_HUB_TRIANGLE_VERTEX);

            boolean withinHubY = rightY < getTurretPose().getY()
                    && getTurretPose().getY() < leftY;

            isBehindHub = Optional.of(behindHubX && withinHubY);

            // === TRIANGLE === (FROM FERRY ZONES):
            // Translation2d turretTranslation = getTurretPose().getTranslation();
            // boolean behindHubX = Field.hubFarLeftCorner.getX() < turretTranslation.getX();
            // 		// && turretTranslation.getX() < Field.hubFarLeftCorner.getX() + Field.hubToleranceX; // With this line the triangle will be cut to more like a trapezoid.
            // // Find point on triangle using the point-slope formula (of the line constructed by the hub corner pose and ferry pose)
            // // y = (slope)(robotX - hubCornerX) + (hubCornerY)
            // // where the slope = (hubCornerY - ferryY)/(hubCornerX - ferryX)
            // double leftY = ((Field.hubFarLeftCorner.getY() - Field.leftFerryZone.getY())/(Field.hubFarLeftCorner.getX() - Field.leftFerryZone.getX())) // (Slope)
            // 				* (turretTranslation.getX() - Field.hubFarLeftCorner.getX()) + Field.hubFarLeftCorner.getY(); // *(robotX - hubCornerX) + (hubCornerY)
            // double rightY = ((Field.hubFarRightCorner.getY() - Field.rightFerryZone.getY())/(Field.hubFarRightCorner.getX() - Field.rightFerryZone.getX())) // (Slope)
            // 				* (turretTranslation.getX() - Field.hubFarRightCorner.getX()) + Field.hubFarRightCorner.getY(); // *(robotX - hubCornerX) + (hubCornerY)
            // leftBehindHubYPlublisher.set(new Pose2d(getTurretPose().getX(), leftY - Field.hubToleranceY, new Rotation2d()));
            // rightBehindHubYPlublisher.set(new Pose2d(getTurretPose().getX(), rightY + Field.hubToleranceY, new Rotation2d()));
            // boolean withinHubY = rightY + Field.hubToleranceY < getTurretPose().getY()
            // 					&& getTurretPose().getY() < leftY - Field.hubToleranceY;
            // return behindHubX && withinHubY;
            // === RECTANGLE ===:
            // Translation2d turretTranslation = getTurretPose().getTranslation();
            // boolean behindHubX = Field.hubFarLeftCorner.getX() < turretTranslation.getX()
            // 		&& turretTranslation.getX() < Field.hubFarLeftCorner.getX() + Field.hubToleranceX;
            // boolean withinHubY = Field.hubFarRightCorner.getY() + Field.hubToleranceY < getTurretPose().getY()
            // 		&& getTurretPose().getY() < Field.hubFarLeftCorner.getY() - Field.hubToleranceY;
            // return behindHubX && withinHubY;
        }

        return isBehindHub.get();
    }

    public boolean isOutsideAllianceZone() {
        if (isOutsideAllianceZone.isEmpty()) {
            isOutsideAllianceZone = Optional.of(getPose().getX() > Field.AllianceRightTrench.rightEdge.getX() + Field.TRENCH_HOOD_TOLERANCE);
        }

        return isOutsideAllianceZone.get();
    }

    public void clearMemoized() {
        isBehindHub = Optional.empty();
        isOutsideAllianceZone = Optional.empty();
        isInOpponentZone = Optional.empty();
        isUnderTrench = Optional.empty();
        isBehindTower = Optional.empty();
    }

    public void teleopInit() {
        for (SwerveModule<TalonFX, TalonFX, CANcoder> module : getModules()) {
            TalonFXConfiguration newConfigs = new TalonFXConfiguration();
            module.getDriveMotor().getConfigurator().refresh(newConfigs);
            newConfigs = newConfigs
                    .withCurrentLimits(new CurrentLimitsConfigs()
                            .withStatorCurrentLimit(80)
                            .withStatorCurrentLimitEnable(true));

            module.getDriveMotor().getConfigurator().apply(newConfigs);
        }
    }

    public void autonInit() {
        for (SwerveModule<TalonFX, TalonFX, CANcoder> module : getModules()) {
            TalonFXConfiguration newConfigs = new TalonFXConfiguration();
            module.getDriveMotor().getConfigurator().refresh(newConfigs);
            newConfigs = newConfigs
                    .withCurrentLimits(new CurrentLimitsConfigs()
                            .withStatorCurrentLimit(120)
                            .withStatorCurrentLimitEnable(true));

            module.getDriveMotor().getConfigurator().apply(newConfigs);
        }
    }

    public double getTotalDriveSupplyCurrent() {
        double total = 0.0;

        for (StatusSignal<Current> s : driveMotorSupplyCurrents) {
            total += Double.max(0, s.getValueAsDouble());
        }

        return total;
    }

    public double getTotalSteerSupplyCurrent() {
        double total = 0.0;

        for (StatusSignal<Current> s : steerMotorSupplyCurrents) {
            total += Double.max(0, s.getValueAsDouble());
        }

        return total;
    }

    public boolean canShootIntoHub() {
        Superstructure superstructure = Superstructure.getInstance();
        SuperstructureState state = superstructure.getState();
        return !isOutsideAllianceZone() || (state == SuperstructureState.KB || state == SuperstructureState.LEFT_CORNER || state == SuperstructureState.RIGHT_CORNER);
    }

    public void periodicAfterScheduler() {
        Pose2d pose = getPose();
        turretPose = new Pose2d(
                pose.getTranslation().plus(
                        Settings.Superstructure.Turret.TURRET_OFFSET.getTranslation().rotateBy(pose.getRotation())),
                pose.getRotation().plus(Turret.getInstance().getAngle()));

        // robotPose.set(pose);

        turret2d.setPose(Robot.isBlue() ? turretPose : Field.transformToOppositeAlliance(turretPose));

        ChassisSpeeds chassisSpeeds = getChassisSpeeds();
        Vector2D fieldRelativeSpeeds = getFieldRelativeSpeeds();
        DogLog.log("Swerve/Velocity Robot Relative X (m per s)", chassisSpeeds.vxMetersPerSecond);
        DogLog.log("Swerve/Velocity Robot Relative Y (m per s)", chassisSpeeds.vyMetersPerSecond);

        DogLog.log("Swerve/Velocity Field Relative X (m per s)", fieldRelativeSpeeds.x);
        DogLog.log("Swerve/Field Relative Rotation", pose.getRotation().getDegrees());
        DogLog.log("Swerve/Velocity Field Relative Y (m per s)", getFieldRelativeSpeeds().y);

        DogLog.log("Swerve/Angular Velocity (rad per s)", chassisSpeeds.omegaRadiansPerSecond);
        DogLog.log("Swerve/Distance From Hub (meters)", Field.HUB_CENTER.getTranslation().getDistance(pose.getTranslation()));

        DogLog.log("Swerve/Pose", pose);

        Field.FIELD2D.getRobotObject().setPose(Robot.isBlue() ? pose : Field.transformToOppositeAlliance(pose));

        if (Robot.getPeriodicCounter() % Settings.LOGGING_FREQUENCY == 0) {
            DogLog.log("Swerve/Robot Accel X", robotAccelerationX.getValueAsDouble() * 9.81);
            DogLog.log("Swerve/Robot Accel Y", robotAccelerationY.getValueAsDouble() * 9.81);

            DogLog.log("Swerve/Failed DAQ Count", this.getState().FailedDaqs);
            DogLog.log("Swerve/CANBus Utiliaztion", Ports.CANIVORE.getStatus().BusUtilization);
            // will confirm whether we are even getting data

            DogLog.log("Superstructure/Turret/Dist From Hub",
                    turretPose.getTranslation().getDistance(Field.HUB_CENTER.getTranslation()));
            DogLog.log("InterpolationTesting/Turret Dist From Hub",
                    turretPose.getTranslation().getDistance(Field.HUB_CENTER.getTranslation()));
            DogLog.log("InterpolationTesting/Turret Dist From Ferry Zone", turretPose.getTranslation()
                    .getDistance(Field.getFerryZonePose(pose.getTranslation()).getTranslation()));

            for (int i = 0; i < 4; i++) {
                String prefix = "Swerve/Modules/Module " + i;
                SwerveModuleState current = getModule(i).getCurrentState();
                SwerveModuleState target = getModule(i).getTargetState();

                DogLog.log(prefix + "/Speed (m per s)", current.speedMetersPerSecond);
                DogLog.log(prefix + "/Target Speed (m per s)", target.speedMetersPerSecond);
                DogLog.log(prefix + "/Angle (deg)", current.angle.getDegrees() % 360);
                DogLog.log(prefix + "/Target Angle (deg)", target.angle.getDegrees() % 360);

                if (Settings.DEBUG_MODE.get()) {
                    DogLog.log(prefix + "/Stator Current",
                            getModule(i).getDriveMotor().getStatorCurrent().getValueAsDouble());
                    DogLog.log(prefix + "/Supply Current",
                            getModule(i).getDriveMotor().getSupplyCurrent().getValueAsDouble());
                }
            }
            Robot.getEnergyUtil().logEnergyUsage(getName() + " Drive", getTotalDriveSupplyCurrent());
            Robot.getEnergyUtil().logEnergyUsage(getName() + " Turn", getTotalSteerSupplyCurrent());

            // CAN SIGNAL LOGGING
            if (Settings.DEBUG_MODE.get() && Robot.getMode() == RobotMode.DISABLED && !Robot.fmsAttached) {
                DogLog.log(
                        "Robot/CAN/Canivore/Front Left Drive Motor Connected? (ID "
                        + String.valueOf(TunerConstants.kFrontLeftDriveMotorId) + ")",
                        getModule(0).getDriveMotor().isConnected());
                DogLog.log(
                        "Robot/CAN/Canivore/Front Left Steer Motor Connected? (ID "
                        + String.valueOf(TunerConstants.kFrontLeftSteerMotorId) + ")",
                        getModule(0).getSteerMotor().isConnected());
                DogLog.log(
                        "Robot/CAN/Canivore/Front Left CANcoder Connected? (ID "
                        + String.valueOf(TunerConstants.kFrontLeftEncoderId) + ")",
                        getModule(0).getEncoder().isConnected());

                DogLog.log(
                        "Robot/CAN/Canivore/Front Right Drive Motor Connected? (ID "
                        + String.valueOf(TunerConstants.kFrontRightDriveMotorId) + ")",
                        getModule(1).getDriveMotor().isConnected());
                DogLog.log(
                        "Robot/CAN/Canivore/Front Right Steer Motor Connected? (ID "
                        + String.valueOf(TunerConstants.kFrontRightSteerMotorId) + ")",
                        getModule(1).getSteerMotor().isConnected());
                DogLog.log(
                        "Robot/CAN/Canivore/Front Right CANcoder Connected? (ID "
                        + String.valueOf(TunerConstants.kFrontRightEncoderId) + ")",
                        getModule(1).getEncoder().isConnected());

                DogLog.log(
                        "Robot/CAN/Canivore/Back Left Drive Motor Connected? (ID "
                        + String.valueOf(TunerConstants.kBackLeftDriveMotorId) + ")",
                        getModule(2).getDriveMotor().isConnected());
                DogLog.log(
                        "Robot/CAN/Canivore/Back Left Steer Motor Connected? (ID "
                        + String.valueOf(TunerConstants.kBackLeftSteerMotorId) + ")",
                        getModule(2).getSteerMotor().isConnected());
                DogLog.log(
                        "Robot/CAN/Canivore/Back Left CANcoder Connected? (ID "
                        + String.valueOf(TunerConstants.kBackLeftEncoderId) + ")",
                        getModule(2).getEncoder().isConnected());

                DogLog.log(
                        "Robot/CAN/Canivore/Back Right Drive Motor Connected? (ID "
                        + String.valueOf(TunerConstants.kBackRightDriveMotorId) + ")",
                        getModule(3).getDriveMotor().isConnected());
                DogLog.log(
                        "Robot/CAN/Canivore/Back Right Steer Motor Connected? (ID "
                        + String.valueOf(TunerConstants.kBackRightSteerMotorId) + ")",
                        getModule(3).getSteerMotor().isConnected());
                DogLog.log(
                        "Robot/CAN/Canivore/Back Right CANcoder Connected? (ID "
                        + String.valueOf(TunerConstants.kBackRightEncoderId) + ")",
                        getModule(3).getEncoder().isConnected());

            }
        }
    }
}
