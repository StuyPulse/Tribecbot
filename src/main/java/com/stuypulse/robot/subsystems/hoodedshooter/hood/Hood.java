/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.hoodedshooter.hood;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.hoodedshooter.HoodAngleCalculator;

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
    
    public static Hood getInstance(){
        return instance;
    }
    
    public enum HoodState {
        STOW,
        FERRY,
        SHOOT,
        HUB,
        LEFT_CORNER,
        RIGHT_CORNER,
        IDLE,
        INTERPOLATION;
    }

    public Hood() {
        state = HoodState.STOW;
    }

    public HoodState getState(){
        return state;
    }

    public void setState(HoodState state){
        this.state = state;
    }

    public Rotation2d getTargetAngle() {
        return switch(state) {
            case STOW -> Settings.HoodedShooter.Angles.MIN_ANGLE;
            case FERRY -> Rotation2d.fromDegrees(30);
            case SHOOT -> Rotation2d.fromDegrees(Settings.HoodedShooter.Angles.SHOOT_ANGLE.get());
            case HUB -> Settings.HoodedShooter.Angles.HUB_ANGLE;
            case LEFT_CORNER -> Settings.HoodedShooter.Angles.LEFT_CORNER_ANGLE;
            case RIGHT_CORNER -> Settings.HoodedShooter.Angles.RIGHT_CORNER_ANGLE;
            case INTERPOLATION -> HoodAngleCalculator.interpolateHoodAngle().get();
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
}