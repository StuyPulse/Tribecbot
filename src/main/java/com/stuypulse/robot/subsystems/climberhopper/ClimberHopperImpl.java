/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.climberhopper;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.climberhopper.ClimberHopper.ClimberHopperState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.Optional;


public class ClimberHopperImpl extends ClimberHopper {
    private final TalonFX motor;
    private final BStream stalling;
    private double voltage;

    private Optional<Double> voltageOverride;

    public ClimberHopperImpl() {
        super();
        motor = new TalonFX(Ports.ClimberHopper.CLIMBER_HOPPER);
        Motors.ClimberHopper.MOTOR.configure(motor);
        
        motor.setPosition(0);
        stalling = BStream.create(() -> motor.getStatorCurrent().getValueAsDouble() > Settings.ClimberHopper.STALL)
            .filtered(new BDebounce.Both(Settings.ClimberHopper.DEBOUNCE));
    }

    @Override 
    public boolean getStalling() {
        return stalling.getAsBoolean();
    }

    @Override
    public double getCurrentHeight() { // TODO: convert motor encoder position to meters somehow
        return this.motor.getPosition().getValueAsDouble() * Settings.ClimberHopper.Constants.POSITION_CONVERSION_FACTOR;
    }

    private boolean isWithinTolerance(double toleranceMeters) {
        return Math.abs(getState().getTargetHeight() - getCurrentHeight()) < toleranceMeters;
    }

    @Override
    public boolean atTargetHeight() {
        return isWithinTolerance(Settings.ClimberHopper.HEIGHT_TOLERANCE_METERS);
    }

    @Override
    public boolean isTrenchSafeRetracted() {
        return Math.abs(ClimberHopperState.HOPPER_DOWN.getTargetHeight() - getCurrentHeight()) < Settings.ClimberHopper.HEIGHT_TOLERANCE_METERS;
    }

    // @Override
    // public void setVoltageOverride(Optional<Double> voltage) {
    //     this.voltageOverride = voltage;
    // }

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
        // if (atTargetHeight() && getState() == ClimberHopperState.HOPPER_DOWN) {
        //     if (voltageOverride.isPresent()) {

        //     }
        // }

        motor.setControl(new VoltageOut(voltage).withEnableFOC(true));

        SmartDashboard.putNumber("ClimberHopper/Voltage", voltage);
        SmartDashboard.putNumber("ClimberHopper/Current", motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("ClimberHopper/Stalling", getStalling());
    }
}