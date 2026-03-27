/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.spindexer;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.Optional;

public abstract class Spindexer extends SubsystemBase {
    private static final Spindexer instance;
    private SpindexerState spindexerState;

    static {
        if (Robot.isReal()) {
            instance = new SpindexerImpl();
        } else {
            instance = new SpindexerSim();
        }
    }

    public static Spindexer getInstance() {
        return instance;
    }

    public enum SpindexerState {
        STOP,
        FORWARD,
        REVERSE;
    }

    public double getTargetRPM() {
        return switch (getState()) {
            case STOP -> 0;
            case FORWARD -> Settings.Spindexer.FORWARD_SPEED;
            case REVERSE -> Settings.Spindexer.REVERSE_SPEED;
        };
    }

    public Spindexer() {
        spindexerState = SpindexerState.STOP;
    }

    public SpindexerState getState() {
        return spindexerState;
    }

    public void setState(SpindexerState state) {
        this.spindexerState = state;
    }

    public abstract boolean atTolerance();
    public abstract boolean canStartIntakeRollers();

    public abstract SysIdRoutine getSysIdRoutine();
    public abstract void setVoltageOverride(Optional<Double> voltage);
    
    public abstract double getCurrentDraw();

    public void periodicAfterScheduler() {
        SmartDashboard.putString("Spindexer/State", getState().name());
        SmartDashboard.putNumber("Spindexer/Target RPM", getTargetRPM());
        SmartDashboard.putBoolean("Spindexer/At Tolerance?", atTolerance());
    }
}