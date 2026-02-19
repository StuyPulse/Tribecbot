/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.hoodedshooter.hood;

import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public abstract class Hood extends SubsystemBase{
    private static final Hood instance;
    
    private HoodState state;

    static {
        instance = new HoodImpl();
    }
    
    public static Hood getInstance() {
        return instance;
    }
    
    public enum HoodState {
        STOW,
        FERRY,
        SHOOT,
        LEFT_CORNER,
        RIGHT_CORNER,
        IDLE;
    }

    public Hood() {
        state = HoodState.STOW;
    }

    public HoodState getState() {
        return state;
    }

    public void setState(HoodState state) {
        this.state = state;
    }

    public Rotation2d getTargetAngle() {
        return switch(state) {
            case STOW -> Constants.HoodedShooter.Hood.MIN_ANGLE;
            case FERRY -> getFerryAngle();
            case SHOOT -> getScoreAngle(); //HoodAngleCalculator.calculateHoodAngleSOTM().get();
            case LEFT_CORNER -> Constants.HoodedShooter.Hood.LEFT_CORNER_ANGLE;
            case RIGHT_CORNER -> Constants.HoodedShooter.Hood.RIGHT_CORNER_ANGLE;
            case IDLE -> getHoodAngle();
        };
    }

    public boolean atTolerance() {
        return Math.abs(getHoodAngle().getDegrees() - getTargetAngle().getDegrees()) < Settings.HoodedShooter.HOOD_TOLERANCE_DEG;
    }

    public abstract Rotation2d getHoodAngle();

    public abstract SysIdRoutine getHoodSysIdRoutine();

    @Override
    public void periodic() {
        SmartDashboard.putString("HoodedShooter/Hood/State", state.name());
        SmartDashboard.putString("States/Hood", state.name());

        SmartDashboard.putNumber("HoodedShooter/Hood/Target Angle", getTargetAngle().getDegrees());
        SmartDashboard.putNumber("HoodedShooter/Hood/Current Angle", getHoodAngle().getDegrees());
    }

    public Rotation2d getFerryAngle() {
        return Rotation2d.fromDegrees(15);
    }

    public Rotation2d getScoreAngle() {
        return Rotation2d.fromDegrees(15);
    }
}