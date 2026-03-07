/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.climber;
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


public class ClimberImpl extends Climber {
    private final Motors.TalonFXConfig climberConfig;

    private final TalonFX motor;
    private final VoltageOut controller;

    private final BStream stalling;
    private double voltage;

    private Optional<Double> voltageOverride;

    public ClimberImpl() {
        super();

        climberConfig = new Motors.TalonFXConfig()
            .withInvertedValue(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)

            .withSupplyCurrentLimitAmps(50.0)
            .withStatorCurrentLimitEnabled(false)
            .withRampRate(Settings.Climber.RAMP_RATE)
            .withSensorToMechanismRatio(Settings.Climber.GEAR_RATIO)
            .withSoftLimits(
                true, true,
                Settings.Climber.MIN_ROTATIONS,
                Settings.Climber.MAX_ROTATIONS);

        motor = new TalonFX(Ports.Climber.CLIMBER_HOPPER, Ports.CANIVORE);
        climberConfig.configure(motor);

        motor.setPosition(Settings.Climber.CLIMBER_DOWN_ROTATIONS);
        stalling = BStream.create(() -> motor.getStatorCurrent().getValueAsDouble() > Settings.Climber.STALL)
            .filtered(new BDebounce.Both(Settings.Climber.DEBOUNCE));

        controller = new VoltageOut(0)
            .withEnableFOC(true);

        voltageOverride = Optional.empty();
    }

    @Override
    public boolean getStalling() {
        return stalling.getAsBoolean();
    }

    @Override
    public double getCurrentRotations() {
        return motor.getPosition().getValueAsDouble();
    }

    private boolean isWithinTolerance(double toleranceMeters) {
        return Math.abs(getState().getTargetHeight() - getCurrentRotations()) < toleranceMeters;
    }

    @Override
    public boolean atTargetHeight() {
        return isWithinTolerance(Settings.Climber.TOLERANCE_ROTATIONS);
    }
    
    public void resetPostionUpper() {
        motor.setPosition(Settings.Climber.MAX_ROTATIONS);
    }
    
    @Override
    public void periodic() {
        super.periodic();
        if (EnabledSubsystems.CLIMBER_HOPPER.get()) {
            ClimberState state = getState();
            double currentPosition = getCurrentRotations();
            double targetPosition = state.getTargetHeight();

            if (voltageOverride.isPresent()) {
                voltage = voltageOverride.get();
            } else if (!atTargetHeight()) {
                    if (state == ClimberState.CLIMBER_DOWN && currentPosition > targetPosition) voltage = -Settings.Climber.MOTOR_VOLTAGE;
                    else if (state == ClimberState.CLIMBER_UP && currentPosition < targetPosition) voltage = Settings.Climber.MOTOR_VOLTAGE;
                    else voltage = 0;
            } else {
                    voltage = 0;
            
            }
            motor.setControl(controller.withOutput(voltage));
        } else {
            motor.stopMotor();
        }
        
        if (Settings.DEBUG_MODE) {
            SmartDashboard.putBoolean("Climber/Stalling", getStalling());
            
            SmartDashboard.putBoolean("Climber/At Target Height?", atTargetHeight());
            SmartDashboard.putNumber("Climber/Voltage", voltage);
            SmartDashboard.putNumber("Climber/Applied Voltage", motor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Climber/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Climber/Stator Current", motor.getStatorCurrent().getValueAsDouble());
            
            SmartDashboard.putNumber("Current Draws/Climber (amps)", motor.getSupplyCurrent().getValueAsDouble());
        }
    }
    
    @Override
    public void setVoltageOverride(Optional<Double> voltage) {
        this.voltageOverride = voltage;
    }
}