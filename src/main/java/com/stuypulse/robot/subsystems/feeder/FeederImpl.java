package com.stuypulse.robot.subsystems.feeder;

import java.util.Optional;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.EnabledSubsystems;
import com.stuypulse.robot.util.SysId;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class FeederImpl extends Feeder {
    private final TalonFX motor;
    private final VelocityVoltage controller;

    private Optional<Double> voltageOverride;

    public FeederImpl() {
        motor = new TalonFX(Ports.Feeder.FEEDER_LEADER);
        Motors.Feeder.motorConfig.configure(motor);

        controller = new VelocityVoltage(getTargetRPM() / 60.0);
        voltageOverride = Optional.empty();
    }

    public double getCurrentRPM() {
        return motor.getVelocity().getValueAsDouble() * Settings.Feeder.SECONDS_IN_A_MINUTE;
    }

    @Override
    public void setVoltageOverride(Optional<Double> voltage) {
        this.voltageOverride = voltage;
    }
    
    @Override
    public void periodic() {
        super.periodic();

        if (EnabledSubsystems.FEEDER.get()) {
            if (getState() == FeederState.STOP) {
                motor.stopMotor();
            } else if (voltageOverride.isPresent()) {
                motor.setVoltage(voltageOverride.get());
            } else {
                motor.setControl(controller.withVelocity(getTargetRPM() / 60.0));
            }
        }

        if (Settings.DEBUG_MODE) {
            SmartDashboard.putNumber("Feeder/Current (amps)", motor.getStatorCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Feeder/Voltage", motor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Feeder/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
        }
    }

    @Override
    public SysIdRoutine getSysIdRoutine() {
        return SysId.getRoutine(
                2,
                8,
                "Feeder",
                voltage -> setVoltageOverride(Optional.of(voltage)),
                () -> motor.getPosition().getValueAsDouble(),
                () -> motor.getVelocity().getValueAsDouble(),
                () -> motor.getMotorVoltage().getValueAsDouble(),
                getInstance());
    }
}