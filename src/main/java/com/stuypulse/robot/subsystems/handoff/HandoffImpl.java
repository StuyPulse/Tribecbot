/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.handoff;

import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.SysId;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.Optional;

public class HandoffImpl extends Handoff {
    private final TalonFX motor;
    private final VelocityVoltage controller;

    private Optional<Double> voltageOverride;

    public HandoffImpl() {
        motor = new TalonFX(Ports.Handoff.HANDOFF);
        Motors.Handoff.HANDOFF.configure(motor);

        controller = new VelocityVoltage(getTargetRPM() / Settings.SECONDS_IN_A_MINUTE);
        voltageOverride = Optional.empty();
    }

    public double getCurrentRPM() {
        return motor.getVelocity().getValueAsDouble() * Settings.SECONDS_IN_A_MINUTE;
    }

    @Override
    public void setVoltageOverride(Optional<Double> voltage) {
        this.voltageOverride = voltage;
    }

    @Override
    public void periodic() {
        super.periodic();

        if (EnabledSubsystems.HANDOFF.get()) {
            if (getState() == HandoffState.STOP) {
                motor.stopMotor();
            } else if (voltageOverride.isPresent()) {
                motor.setVoltage(voltageOverride.get());
            } else {
                motor.setControl(controller.withVelocity(getTargetRPM() / 60.0));
            }
        }

        if (Settings.DEBUG_MODE) {
            SmartDashboard.putNumber("Handoff/Current (amps)", motor.getStatorCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Handoff/Voltage", motor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Handoff/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
        }
    }

    @Override
    public SysIdRoutine getSysIdRoutine() {
        return SysId.getRoutine(
                2,
                8,
                "Handoff",
                voltage -> setVoltageOverride(Optional.of(voltage)),
                () -> motor.getPosition().getValueAsDouble(),
                () -> motor.getVelocity().getValueAsDouble(),
                () -> motor.getMotorVoltage().getValueAsDouble(),
                getInstance());
    }
}