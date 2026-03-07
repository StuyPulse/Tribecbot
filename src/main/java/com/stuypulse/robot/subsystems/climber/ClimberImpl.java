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
    private final Motors.TalonFXConfig climberHopperConfig;

    private final TalonFX motor;
    private final VoltageOut controller;

    private final BStream stalling;
    private double voltage;

    private Optional<Double> voltageOverride;

    public ClimberImpl() {
        super();

        climberHopperConfig = new Motors.TalonFXConfig()
            .withInvertedValue(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)

            .withSupplyCurrentLimitAmps(50.0)
            .withStatorCurrentLimitEnabled(false)
            .withRampRate(Settings.ClimberHopper.RAMP_RATE)
            .withSensorToMechanismRatio(Settings.ClimberHopper.GEAR_RATIO)
            .withSoftLimits(
                true, true,
                Settings.ClimberHopper.MIN_ROTATIONS,
                Settings.ClimberHopper.MAX_ROTATIONS);

        motor = new TalonFX(Ports.ClimberHopper.CLIMBER_HOPPER, Ports.CANIVORE);
        climberHopperConfig.configure(motor);

        motor.setPosition(Settings.ClimberHopper.CLIMBER_DOWN_ROTATIONS);
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
    public double getCurrentRotations() {
        return motor.getPosition().getValueAsDouble();
    }

    private boolean isWithinTolerance(double toleranceMeters) {
        return Math.abs(getState().getTargetHeight() - getCurrentRotations()) < toleranceMeters;
    }

    @Override
    public boolean atTargetHeight() {
        return isWithinTolerance(Settings.ClimberHopper.TOLERANCE_ROTATIONS);
    }
    
    public void resetPostionUpper() {
        motor.setPosition(Settings.ClimberHopper.MAX_ROTATIONS);
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
                    if (state == ClimberState.CLIMBER_DOWN && currentPosition > targetPosition) voltage = -Settings.ClimberHopper.MOTOR_VOLTAGE;
                    else if (state == ClimberState.CLIMBER_UP && currentPosition < targetPosition) voltage = Settings.ClimberHopper.MOTOR_VOLTAGE;
                    else voltage = 0;
            } else {
                    voltage = 0;
            
            }
            motor.setControl(controller.withOutput(voltage));
        } else {
            motor.stopMotor();
        }
        
        if (Settings.DEBUG_MODE) {
            SmartDashboard.putBoolean("ClimberHopper/Stalling", getStalling());
            
            SmartDashboard.putBoolean("Climber/At Target Height?", atTargetHeight());
            SmartDashboard.putNumber("ClimberHopper/Voltage", voltage);
            SmartDashboard.putNumber("ClimberHopper/Applied Voltage", motor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("ClimberHopper/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("ClimberHopper/Stator Current", motor.getStatorCurrent().getValueAsDouble());
            
            SmartDashboard.putNumber("Current Draws/ClimberHopper (amps)", motor.getSupplyCurrent().getValueAsDouble());
        }
    }
    
    @Override
    public void setVoltageOverride(Optional<Double> voltage) {
        this.voltageOverride = voltage;
    }
}