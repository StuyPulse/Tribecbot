/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.superstructure.turret;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.superstructure.SOTMCalculator;
import com.stuypulse.robot.util.superstructure.TurretAngleCalculator;
import com.stuypulse.robot.util.superstructure.VisualizerTurret;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Vector2D;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public abstract class Turret extends SubsystemBase {
    private static final Turret instance;
    private TurretState state;
    private Vector2D driverInput;

    static {
        instance = Robot.isReal() ? new TurretImpl() : new TurretSim();
    }

    public static Turret getInstance() {
        return instance;
    }

    public Turret() {
        driverInput = Vector2D.kOrigin;
        state = TurretState.SCORE;
    }

    public void setDriverInput(Gamepad gamepad) {
        this.driverInput = gamepad.getLeftStick();
    }

    public enum TurretState {
        IDLE,
        ZERO,
        SCORE,
        SOTM,
        FOTM,
        FERRY,
        LEFT_CORNER,
        RIGHT_CORNER,
        KB,
        TESTING;
    }

    public Rotation2d getTargetAngle() {
        return switch (getState()) {
            case IDLE -> getAngle(); 
            case ZERO -> Rotation2d.kZero;
            case SCORE -> getScoringAngle();
            case SOTM -> SOTMCalculator.calculateTurretAngleSOTM();
            case FOTM -> SOTMCalculator.calculateTurretAngleFOTM();
            case FERRY -> getFerryAngle();
            case LEFT_CORNER -> Settings.Superstructure.Turret.LEFT_CORNER;
            case RIGHT_CORNER -> Settings.Superstructure.Turret.RIGHT_CORNER;
            case KB -> Settings.Superstructure.Turret.KB;
            case TESTING -> driverInputToAngle();
        };
    }

    public Rotation2d driverInputToAngle() {
        DogLog.log("Superstructure/Turret/Driver Input", driverInput.x);
        return Rotation2d.fromDegrees(driverInput.x * 180); 
    }
 
    public boolean atTolerance() {
        double error = getAngle().minus(getTargetAngle()).getRotations();
        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

        double tolerance = switch (state) {
            case SOTM -> 
                swerve.getTurretPose().getTranslation().getDistance(Field.HUB_CENTER.getTranslation()) > 
                Settings.Superstructure.Turret.SOTM_TOLERANCE_THRESHOLD_METERS.get() ?
                Settings.Superstructure.Turret.SOTM_TOLERANCE_CLOSE.get() / 360.0 :
                Settings.Superstructure.Turret.SOTM_TOLERANCE_FAR.get() / 360.0;
            case FOTM -> Settings.Superstructure.Turret.FOTM_TOLERANCE.getRotations();
            default  -> Settings.Superstructure.Turret.TOLERANCE.getRotations();
        };

        return Math.abs(error) < tolerance;
    }

    public Rotation2d getScoringAngle() {
        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

        Translation2d target = Field.getHubPose().getTranslation();
        Translation2d turret = swerve.getTurretPose().getTranslation();

        return TurretAngleCalculator.getPointAtTargetAngle(target, turret, swerve.getPose().getRotation());
    }

    public Rotation2d getFerryAngle() {
        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

        Pose2d robot = swerve.getPose();
        Translation2d target = Field.getFerryZonePose(robot.getTranslation()).getTranslation();
        Translation2d turret = swerve.getTurretPose().getTranslation();

        return TurretAngleCalculator.getPointAtTargetAngle(target, turret, robot.getRotation());
    }

    public abstract Rotation2d getAngle();

    public abstract SysIdRoutine getSysIdRoutine();

    public abstract void seedTurret();
    public abstract void zeroEncoders();

    public abstract boolean isWrapping();
    public abstract double getCurrentDraw();

    public void setState(TurretState state) {
        this.state = state;
    }

    public TurretState getState() {
        return this.state;
    }

    public void periodicAfterScheduler() {
        DogLog.log("Superstructure/Turret/State", state.name());
        
        DogLog.log("Superstructure/Turret/Target Angle", getTargetAngle().getDegrees());
        DogLog.log("Superstructure/Turret/Current Angle", getAngle().getDegrees());

        if (Settings.DEBUG_MODE.get()) {
            if (EnabledSubsystems.TURRET.get()) {
                VisualizerTurret.getInstance().updateTurretAngle(getAngle().plus((Robot.isBlue() ? Rotation2d.kZero : Rotation2d.k180deg)), atTolerance());
            }
            else {
                VisualizerTurret.getInstance().updateTurretAngle(new Rotation2d(), false);
            }
        }
    }
}