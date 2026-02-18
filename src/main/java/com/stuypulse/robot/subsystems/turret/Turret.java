/************************ PROJECT ALPHA *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.turret;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Vector2D;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.turret.TurretVisualizer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public abstract class Turret extends SubsystemBase {
    private static final Turret instance;
    private TurretState state;
    private Vector2D driverInput;
    private FieldObject2d turret2d;

    static {
        instance = Robot.isReal() ? new TurretImpl() : new TurretSim();
    }

    public static Turret getInstance() {
        return instance;
    }

    public Turret() {
        driverInput = new Vector2D(0, 0);
        state = TurretState.IDLE;

        turret2d = Field.FIELD2D.getObject("Turret 2D");
    }

    public void setDriverInput(Gamepad gamepad) {
        this.driverInput = gamepad.getLeftStick();
    }

    public enum TurretState {
        IDLE,
        ZERO,
        SHOOTING,
        FERRYING,
        HUB,
        LEFT_CORNER,
        RIGHT_CORNER,
        TESTING;
    }

    public Rotation2d getTargetAngle() {
        return switch (getState()) {
            case IDLE -> getAngle(); 
            case ZERO -> Rotation2d.kZero;
            case FERRYING -> Rotation2d.fromDegrees(0); //TODO: CHANGE TO getFerryAngle();
            case SHOOTING -> getScoringAngle();
            case HUB -> Settings.Turret.HUB;
            case LEFT_CORNER -> Settings.Turret.LEFT_CORNER;
            case TESTING -> driverInputToAngle();
            case RIGHT_CORNER -> Settings.Turret.RIGHT_CORNER;
        };
    }

    public Rotation2d driverInputToAngle() {
        SmartDashboard.putNumber("Turret/Driver Input", driverInput.x);
        return Rotation2d.fromDegrees(driverInput.x * 180); 
    }
 
    public boolean atTargetAngle() {
        return Math.abs(getAngle().minus(getTargetAngle()).getDegrees()) < Settings.Turret.TOLERANCE_DEG;
    }

    public Rotation2d getScoringAngle() {
        return getPointAtTargetAngle(Field.getHubPose());
    }

    public Rotation2d getFerryAngle() {
        Pose2d robot = CommandSwerveDrivetrain.getInstance().getPose();
        return getPointAtTargetAngle(Field.getFerryZonePose(robot.getTranslation()));
    }

    public abstract Rotation2d getAngle();

    public abstract SysIdRoutine getSysIdRoutine();

    public abstract void seedTurret();
   
    public void setState(TurretState state) {
        this.state = state;
    }

    public TurretState getState() {
        return this.state;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Turret/State", state.name());
        SmartDashboard.putString("States/Turret", state.name());
        SmartDashboard.putNumber("Turret/Target Angle", getTargetAngle().getDegrees());

        Pose2d robotPose = CommandSwerveDrivetrain.getInstance().getPose();

        Pose2d turretTranslation = robotPose.plus(Constants.Turret.TURRET_OFFSET);
        Rotation2d turretRotation = robotPose.getRotation().plus(getAngle());

        Pose2d turretPose = new Pose2d(turretTranslation.getTranslation(), turretRotation);

        turret2d.setPose(Robot.isBlue() ? turretPose : Field.transformToOppositeAlliance(turretPose));

        if (Settings.DEBUG_MODE) {
            if (Settings.EnabledSubsystems.TURRET.get()) {
                TurretVisualizer.getInstance().updateTurretAngle(getAngle().plus((Robot.isBlue() ? Rotation2d.kZero : Rotation2d.k180deg)), atTargetAngle());
            }
            else {
                TurretVisualizer.getInstance().updateTurretAngle(new Rotation2d(), false);
            }
        }
    }

    // Should match implementation on mini turret
    // Current logic is as of 2/15
    public Rotation2d getPointAtTargetAngle(Pose2d targetPose) {
        Pose2d robotPose = CommandSwerveDrivetrain.getInstance().getPose();
        Pose2d turretPose = robotPose.plus(Constants.Turret.TURRET_OFFSET);

        // Vector2D robot = new Vector2D(robotPose.getTranslation());
        Vector2D turret = new Vector2D(turretPose.getTranslation());
        Vector2D target = new Vector2D(targetPose.getTranslation());

        Vector2D turretToTarget = target.sub(turret);
        Vector2D zeroVector = new Vector2D(robotPose.getRotation().getCos(), robotPose.getRotation().getSin());

        // https://www.youtube.com/watch?v=_VuZZ9_58Wg
        double crossProduct = zeroVector.x * turretToTarget.y - zeroVector.y * turretToTarget.x;
        double dotProduct = zeroVector.dot(turretToTarget);

        SmartDashboard.putNumber("Turret/Turret to Target Vector X", turretToTarget.x);
        SmartDashboard.putNumber("Turret/Turret to Target Vector Y", turretToTarget.y);
        SmartDashboard.putNumber("Turret/Target Pose X", targetPose.getX());
        SmartDashboard.putNumber("Turret/Target Pose Y", targetPose.getY());
        // SmartDashboard.putNumber("Turret/Robot to Target Vector X", robotToHub.x);
        // SmartDashboard.putNumber("Turret/Robot to Target Vector Y", robotToHub.y);
        SmartDashboard.putNumber("Turret/Zero Vector X", zeroVector.x);
        SmartDashboard.putNumber("Turret/Zero Vector Y", zeroVector.y);

        Rotation2d targetAngle = (Robot.isReal() ?
            Rotation2d.fromRadians(-Math.atan2(crossProduct, dotProduct)) :
            Rotation2d.fromRadians(Math.atan2(crossProduct, dotProduct)));

        return targetAngle;
    }
}