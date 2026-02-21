/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.climberhopper;


import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ClimberHopperSim extends ClimberHopper {
    
    private final ElevatorSim sim;
    private final ClimberHopperVisualizer visualizer;
    private double voltage;

    public ClimberHopperSim() {
        visualizer = ClimberHopperVisualizer.getInstance();

        sim = new ElevatorSim(
            DCMotor.getKrakenX60(1),
            Settings.ClimberHopper.Constants.GEAR_RATIO,
            Settings.ClimberHopper.Constants.MASS_KG,
            Settings.ClimberHopper.Constants.DRUM_RADIUS_METERS,
            Settings.ClimberHopper.Constants.MIN_HEIGHT_METERS,
            Settings.ClimberHopper.Constants.MAX_HEIGHT_METERS,
            false,
            Settings.ClimberHopper.Constants.MIN_HEIGHT_METERS
        );
    }

    public boolean getStalling() {
        return sim.getCurrentDrawAmps() > Settings.ClimberHopper.STALL;
    }

    @Override
    public double getCurrentHeight() {
        return sim.getPositionMeters();
    }

    private boolean isWithinTolerance(double toleranceMeters) {
        return Math.abs(getState().getTargetHeight() - getCurrentHeight()) < toleranceMeters;
    }

    @Override
    public boolean atTargetHeight() {
        return isWithinTolerance(Settings.ClimberHopper.HEIGHT_TOLERANCE_METERS);
    }

    // @Override
    // public void setVoltageOverride(Optional<Double> voltage) {
    //     this.voltageOverride = voltage;
    // }

    @Override
    public boolean isTrenchSafeRetracted() {
        return getState() == ClimberHopperState.HOPPER_DOWN && atTargetHeight();
    }

    @Override
    public void periodic() {
        super.periodic();

        if (!atTargetHeight()) {
            if (getCurrentHeight() < getState().getTargetHeight()) {
                voltage = Settings.ClimberHopper.MOTOR_VOLTAGE;
            } else {
                voltage = - Settings.ClimberHopper.MOTOR_VOLTAGE;
            }
        } else {
            voltage = 0;
        }

        // TODO: Figure out some way to reset the encoder reading when stall

        sim.setInputVoltage(voltage);

        SmartDashboard.putNumber("ClimberHopper/Voltage", voltage);
        SmartDashboard.putNumber("ClimberHopper/Current", sim.getCurrentDrawAmps());
        SmartDashboard.putBoolean("ClimberHopper/Stalling", getStalling());
        SmartDashboard.putNumber("ClimberHopper/Height", getCurrentHeight());
        visualizer.update(getCurrentHeight());

        sim.update(0.02);
    }
}