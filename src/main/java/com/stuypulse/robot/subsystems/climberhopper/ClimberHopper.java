/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.climberhopper;

import java.util.Optional;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ClimberHopper extends SubsystemBase {
    private static final ClimberHopper instance;
    
    static {
        if (Robot.isReal()) {
            instance = new ClimberHopperImpl();
        } else {
            instance = new ClimberHopperSim();
        }
    }

    public static ClimberHopper getInstance() {
        return instance;
    }

    public enum ClimberHopperState {
        CLIMBER_UP(Settings.ClimberHopper.CLIMBER_UP_HEIGHT_METERS),
        CLIMBER_DOWN(Settings.ClimberHopper.CLIMBER_DOWN_HEIGHT_METERS),
        HOPPER_UP(Settings.ClimberHopper.HOPPER_UP_HEIGHT_METERS),
        HOPPER_DOWN(Settings.ClimberHopper.HOPPER_DOWN_HEIGHT_METERS);
    
        private double targetHeight;
        
        private ClimberHopperState(double targetHeight) {
            this.targetHeight = targetHeight;
        }
        
        public double getTargetHeight() {
            return targetHeight;
        }

    }
    
    protected ClimberHopperState state;

    protected ClimberHopper() {
        this.state = ClimberHopperState.CLIMBER_UP;
    }
    
    public ClimberHopperState getState() {
        return state;
    }

    public void setState(ClimberHopperState state) {
        this.state = state;
    }

    public abstract boolean getStalling();
    public abstract double getCurrentHeight();
    public abstract boolean atTargetHeight();

    public abstract void setVoltageOverride(Optional<Double> voltage);

    @Override
    public void periodic() {
        SmartDashboard.putString("ClimberHopper/State", getState().toString());
    }
}