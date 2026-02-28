/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.climberhopper;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import java.util.Optional;


public class ClimberHopperImpl extends ClimberHopper {
    private final Motors.TalonFXConfig climberHopperConfig;

    private final TalonFX motor;
    private final VoltageOut controller;

    private final BStream stalling;
    private double voltage;

    private Optional<Double> voltageOverride;

    public ClimberHopperImpl() {
        super();

        climberHopperConfig = new Motors.TalonFXConfig()
            .withInvertedValue(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
            .withCurrentLimitAmps(50.0)
            .withSupplyCurrentLimitAmps(50.0)
            .withRampRate(Settings.ClimberHopper.RAMP_RATE)
            .withSoftLimits(
                false, false,
                Settings.ClimberHopper.ROTATIONS_AT_BOTTOM + Settings.ClimberHopper.Constants.NUM_ROTATIONS_TO_REACH_TOP,
                Settings.ClimberHopper.ROTATIONS_AT_BOTTOM);

        motor = new TalonFX(Ports.ClimberHopper.CLIMBER_HOPPER, Ports.CANIVORE);
        climberHopperConfig.configure(motor);

        motor.setPosition(Settings.ClimberHopper.ROTATIONS_AT_BOTTOM);
        stalling = BStream.create(() -> motor.getStatorCurrent().getValueAsDouble() > Settings.ClimberHopper.STALL)
            .filtered(new BDebounce.Both(Settings.ClimberHopper.DEBOUNCE));

        controller = new VoltageOut(0)
            .withEnableFOC(true);

        voltageOverride = Optional.empty();
    }

    @Override
    public boolean getStalling() {
        return stalling.getAsBoolean();
    }

    @Override
    public double getCurrentHeight() {
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
    public void setVoltageOverride(Optional<Double> voltage) {
        this.voltageOverride = voltage;
    }

    public void resetPostionUpper() {
        motor.setPosition(Settings.ClimberHopper.ROTATIONS_AT_BOTTOM + Settings.ClimberHopper.Constants.NUM_ROTATIONS_TO_REACH_TOP);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (voltageOverride.isPresent()) {
            voltage = voltageOverride.get();
        } else {
            if (getState() == ClimberHopperState.CLIMBER_DOWN) voltage = -Settings.ClimberHopper.MOTOR_VOLTAGE;
            else if (getState() == ClimberHopperState.CLIMBER_UP) voltage = Settings.ClimberHopper.MOTOR_VOLTAGE;
            else voltage = 0;
        }

        if (EnabledSubsystems.CLIMBER_HOPPER.get()) {
            motor.setControl(controller.withOutput(voltage));
        } else {
            motor.stopMotor();
        }

        SmartDashboard.putBoolean("ClimberHopper/Stalling", getStalling());

        if (Settings.DEBUG_MODE) {
            SmartDashboard.putNumber("ClimberHopper/Current Height", getCurrentHeight());
            SmartDashboard.putNumber("ClimberHopper/Voltage", voltage);
            SmartDashboard.putNumber("ClimberHopper/Applied Voltage", motor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("ClimberHopper/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("ClimberHopper/Stator Current", motor.getStatorCurrent().getValueAsDouble());
        }
    }
}