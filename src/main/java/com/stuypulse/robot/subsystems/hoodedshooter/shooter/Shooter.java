/************************ PROJECT ALPHA *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.hoodedshooter.shooter;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.hoodedshooter.HoodedShooter;
import com.stuypulse.robot.util.hoodedshooter.HoodAngleCalculator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public abstract class Shooter extends SubsystemBase {
    private static final Shooter instance;

    private ShooterState state;

    static {
        if (Robot.isReal()) {
            instance = new ShooterImpl();
        } else {
            instance = new ShooterSim();
        }
    }

    public static Shooter getInstance() {
        return instance;
    }

    public enum ShooterState {
        STOP,
        SHOOT,
        FERRY,
        REVERSE,
        HUB,
        LEFT_CORNER,
        RIGHT_CORNER,  
        INTERPOLATION;
    }

    public Shooter() {
        state = ShooterState.SHOOT;
    }

    public void setState(ShooterState state) {
        this.state = state;
    }

    public ShooterState getState() {
        return state;
    }

    public double getTargetRPM() {
        return switch(state) {
            case STOP -> 0;
            case SHOOT -> getShootRPM();
            case FERRY -> HoodAngleCalculator.interpolateFerryingRPM().get();
            case REVERSE -> Settings.HoodedShooter.RPMs.REVERSE;
            case HUB -> Settings.HoodedShooter.RPMs.HUB_RPM;
            case LEFT_CORNER -> Settings.HoodedShooter.RPMs.LEFT_CORNER_RPM;
            case RIGHT_CORNER -> Settings.HoodedShooter.RPMs.RIGHT_CORNER_RPM;
            case INTERPOLATION -> HoodAngleCalculator.interpolateShooterRPM().get();
        };
    }

    public double getShootRPM() {
        return Settings.HoodedShooter.RPMs.SHOOT_RPM.get(); // will return different speeds in future based on distance to hub
    }

    public double getFerryRPM() {
        return Settings.HoodedShooter.RPMs.FERRY_RPM.get(); 
    }

    public boolean atTolerance() {
        double diff = Math.abs(getTargetRPM() - getShooterRPM());
        return diff < Settings.HoodedShooter.SHOOTER_TOLERANCE_RPM;
    }

    public abstract double getShooterRPM();

    public abstract SysIdRoutine getShooterSysIdRoutine();

    @Override
    public void periodic() {
        SmartDashboard.putString("HoodedShooter/Shooter/State", state.name());
        SmartDashboard.putString("States/Shooter", state.name());

        SmartDashboard.putNumber("HoodedShooter/Shooter/Current RPM", getShooterRPM());
        SmartDashboard.putNumber("HoodedShooter/Shooter/Target RPM", getTargetRPM());

        SmartDashboard.putNumber("InterpolationTesting/Shooter Interpolated Target RPM", HoodAngleCalculator.interpolateShooterRPM().get());
    }
}
