/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.superstructure.shooter;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.superstructure.InterpolationCalculator;
import com.stuypulse.robot.util.superstructure.SOTMCalculator;

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
        KB,
        LEFT_CORNER,
        RIGHT_CORNER,  
        INTERPOLATION,
        SOTM,
        FOTM;
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
        // if(Settings.Superstructure.Shooter.RPM.OVERRIDEN.get()) {
        //     return Settings.Superstructure.Shooter.RPM.OVERRIDE_VALUE.get();
        // }
        
        return switch(state) {
            case STOP -> 0;
            case SHOOT -> getShootRPM();
            case FERRY -> InterpolationCalculator.interpolateFerryingInfo().targetRPM();
            case REVERSE -> Settings.Superstructure.Shooter.RPM.REVERSE;
            case KB -> Settings.Superstructure.Shooter.RPM.KB;
            case LEFT_CORNER -> Settings.Superstructure.Shooter.RPM.LEFT_CORNER;
            case RIGHT_CORNER -> Settings.Superstructure.Shooter.RPM.RIGHT_CORNER;
            case INTERPOLATION -> InterpolationCalculator.interpolateShotInfo().targetRPM();
            case SOTM -> SOTMCalculator.calculateShooterRPMSOTM();
            case FOTM -> SOTMCalculator.calculateShooterRPMFOTM();
        };
    }

    public double getShootRPM() {
        return Settings.Superstructure.Shooter.RPM.SHOOT.get(); // Adjustable RPM on Glass
    }

    public boolean atTolerance() {
        double error = getTargetRPM() - getRPM();

        double toleranceHigh = switch (state) {
            case SOTM -> Settings.Superstructure.SHOOTER_SOTM_TOLERANCE_RPM_HIGH;
            case FOTM -> Settings.Superstructure.SHOOTER_FOTM_TOLERANCE_RPM_HIGH;
            default  -> Settings.Superstructure.SHOOTER_TOLERANCE_RPM_HIGH;
        };

        double toleranceLow = switch (state) {
            case SOTM -> Settings.Superstructure.SHOOTER_SOTM_TOLERANCE_RPM_LOW;
            case FOTM -> Settings.Superstructure.SHOOTER_FOTM_TOLERANCE_RPM_LOW;
            default  -> Settings.Superstructure.SHOOTER_TOLERANCE_RPM_LOW;
        };

        return error > -toleranceLow && error < toleranceHigh;
    }

    public abstract double getRPM();

    public abstract SysIdRoutine getShooterSysIdRoutine();
    
    public abstract double getCurrentDraw();

    public void periodicAfterScheduler() {
        SmartDashboard.putString("Superstructure/Shooter/State", state.name());

        SmartDashboard.putNumber("Superstructure/Shooter/Current RPM (Leader)", getRPM());
        SmartDashboard.putNumber("Superstructure/Shooter/Target RPM", getTargetRPM());
    }
}
