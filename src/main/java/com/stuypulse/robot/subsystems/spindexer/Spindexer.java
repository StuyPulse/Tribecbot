/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.spindexer;

import java.util.Optional;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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

    public double getTargetDutyCycle() {
        return switch (getState()) {
            case STOP -> 0;
            case FORWARD -> Settings.Spindexer.FORWARD_DUTY_CYCLE;//((Robot.getPeriodicCounter() % Settings.Spindexer.ANTI_POPCORN_FREQ <= Settings.Spindexer.ANTI_POPCORN_LENGTH) ? Settings.Spindexer.ANTI_POPCORN_DUTY_CYCLE : Settings.Spindexer.FORWARD_DUTY_CYCLE);
            case REVERSE -> Settings.Spindexer.REVERSE_DUTY_CYCLE;
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

    // public abstract boolean atTolerance();
    // public abstract boolean canStartIntakeRollers();

    public abstract SysIdRoutine getSysIdRoutine();
    public abstract void setVoltageOverride(Optional<Double> voltage);
    
    public abstract double getCurrentDraw();

    public void periodicAfterScheduler() {
        DogLog.log("Spindexer/State", getState().name());
        DogLog.log("Spindexer/Target Duty Cycle", getTargetDutyCycle());
    }
}