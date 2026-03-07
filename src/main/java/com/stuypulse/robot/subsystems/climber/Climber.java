/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.climber;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

public abstract class Climber extends SubsystemBase {
    private static final Climber instance;
    
    static {
        if (Robot.isReal()) {
            instance = new ClimberImpl();
        } else {
           instance = new ClimberSim();
        }
    }

    public static Climber getInstance() {
        return instance;
    }

    public enum ClimberState {
        CLIMBER_UP(Settings.Climber.CLIMBER_UP_ROTATIONS),
        CLIMBER_DOWN(Settings.Climber.CLIMBER_DOWN_ROTATIONS),
        CLIMBER_CLIMB(Settings.Climber.CLIMBER_CLIMB_ROTATIONS),
        CLIMBER_STOP(0.0); //TODO: remove, for debug only
    
        private double targetHeight;
        
        private ClimberState(double targetHeight) {
            this.targetHeight = targetHeight;
        }
        
        public double getTargetHeight() {
            return targetHeight;
        }

    }
    
    private ClimberState state;

    protected Climber() {
        this.state = ClimberState.CLIMBER_DOWN;
    }
    
    public ClimberState getState() {
        return state;
    }

    public void setState(ClimberState state) {
        this.state = state;
    }

    public abstract boolean getStalling();
    public abstract double getCurrentRotations();
    public abstract boolean atTargetHeight();
    
    /**
     * Resets the encoder postition to the upper hardstop
     */
    public abstract void resetPostionUpper();

    public abstract void setVoltageOverride(Optional<Double> voltage);

    @Override
    public void periodic() {
        SmartDashboard.putString("Climber/State", getState().toString());
    }
}