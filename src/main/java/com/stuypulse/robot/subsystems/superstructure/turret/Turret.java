/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.superstructure.turret;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Vector2D;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.superstructure.SOTMCalculator;
import com.stuypulse.robot.util.superstructure.TurretAngleCalculator;
import com.stuypulse.robot.util.superstructure.TurretVisualizer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        state = TurretState.IDLE;
    }

    public void setDriverInput(Gamepad gamepad) {
        this.driverInput = gamepad.getLeftStick();
    }

    public enum TurretState {
        IDLE,
        ZERO,
        SHOOT,
        SOTM,
        FERRY,
        LEFT_CORNER,
        RIGHT_CORNER,
        TESTING;
    }

    public Rotation2d getTargetAngle() {
        return switch (getState()) {
            case IDLE -> getAngle(); 
            case ZERO -> Rotation2d.kZero;
            case SHOOT -> getScoringAngle();
            case SOTM -> SOTMCalculator.calculateTurretAngleSOTM();
            case FERRY -> getFerryAngle();
            case LEFT_CORNER -> Settings.Superstructure.Turret.LEFT_CORNER;
            case RIGHT_CORNER -> Settings.Superstructure.Turret.RIGHT_CORNER;
            case TESTING -> driverInputToAngle();
        };
    }

    public Rotation2d driverInputToAngle() {
        SmartDashboard.putNumber("SuperStructure/Turret/Driver Input", driverInput.x);
        return Rotation2d.fromDegrees(driverInput.x * 180); 
    }
 
    public boolean atTargetAngle() {
        double error = getAngle().minus(getTargetAngle()).getRotations();
        return Math.abs(error) < Settings.Superstructure.Turret.TOLERANCE.getRotations();
    }

    public Rotation2d getScoringAngle() {
        Translation2d target = Field.getHubPose().getTranslation();
        Translation2d turret = CommandSwerveDrivetrain.getInstance().getTurretPose().getTranslation();
        return TurretAngleCalculator.getPointAtTargetAngle(target, turret);
    }

    public Rotation2d getFerryAngle() {
        Pose2d robot = CommandSwerveDrivetrain.getInstance().getPose();
        Translation2d target = Field.getFerryZonePose(robot.getTranslation()).getTranslation();
        Translation2d turret = CommandSwerveDrivetrain.getInstance().getTurretPose().getTranslation();

        return TurretAngleCalculator.getPointAtTargetAngle(target, turret);
    }

    public abstract Rotation2d getAngle();

    public abstract SysIdRoutine getSysIdRoutine();

    public abstract void seedTurret();
    public abstract void zeroEncoders();
   
    public void setState(TurretState state) {
        this.state = state;
    }

    public TurretState getState() {
        return this.state;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("SuperStructure/Turret/State", state.name());
        SmartDashboard.putString("States/Turret", state.name());
        SmartDashboard.putNumber("SuperStructure/Turret/Target Angle", getTargetAngle().getDegrees());
        SmartDashboard.putBoolean("SuperStructure/Turret/At Target Angle?", atTargetAngle());

        if (Settings.DEBUG_MODE) {
            if (EnabledSubsystems.TURRET.get()) {
                TurretVisualizer.getInstance().updateTurretAngle(getAngle().plus((Robot.isBlue() ? Rotation2d.kZero : Rotation2d.k180deg)), atTargetAngle());
            }
            else {
                TurretVisualizer.getInstance().updateTurretAngle(new Rotation2d(), false);
            }
        }
    }
}