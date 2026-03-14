/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.handoff;

import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.spindexer.Spindexer.SpindexerState;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.SysId;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import java.util.Optional;

public class HandoffImpl extends Handoff {
    private final Motors.TalonFXConfig handoffConfig;

    private final TalonFX motor;
    private final VelocityVoltage controller;

    private Optional<Double> voltageOverride;
    private BStream isStalling;

    public HandoffImpl() {
        handoffConfig = new Motors.TalonFXConfig()
            .withInvertedValue(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
            
            .withSupplyCurrentLimitAmps(80.0)
            .withStatorCurrentLimitEnabled(false)
            .withRampRate(0.25)
            
            .withPIDConstants(Gains.Handoff.kP, Gains.Handoff.kI, Gains.Handoff.kD, 0)
            .withFFConstants(Gains.Handoff.kS, Gains.Handoff.kV, Gains.Handoff.kA, 0)
            
            .withSensorToMechanismRatio(Settings.Handoff.GEAR_RATIO);

        motor = new TalonFX(Ports.Handoff.HANDOFF, Ports.RIO);
        handoffConfig.configure(motor);

        controller = new VelocityVoltage(getTargetRPM() / Settings.SECONDS_IN_A_MINUTE).withEnableFOC(true);
        voltageOverride = Optional.empty();

        isStalling = BStream.create(() -> motor.getSupplyCurrent().getValueAsDouble() > Settings.Handoff.HANDOFF_STALL_CURRENT.getAsDouble())
            .filtered(new BDebounce.Both(0.5));
    }

    @Override
    public boolean isHandoffStalling() {
        return isStalling.get();
    }

    public double getCurrentRPM() {
        return motor.getVelocity().getValueAsDouble() * Settings.SECONDS_IN_A_MINUTE * Settings.Handoff.GEAR_RATIO;
    }

    public boolean shouldStop() {
        boolean isStopState = getState() == HandoffState.STOP;
        boolean isTurretWrapping = Superstructure.getInstance().isTurretWrapping();
        boolean isBehindHubWhileFerrying = Superstructure.getInstance().getState() == SuperstructureState.FOTM && CommandSwerveDrivetrain.getInstance().isBehindHub();

        return isStopState || isTurretWrapping || isBehindHubWhileFerrying;
    }
    
    @Override
    public void periodic() {
        super.periodic();
        
        if (EnabledSubsystems.HANDOFF.get() && getState() != HandoffState.STOP) {
            if (voltageOverride.isPresent()) {
                motor.setVoltage(voltageOverride.get());
            } else if (shouldStop()) {
                motor.stopMotor();
            } else {
                motor.setControl(controller.withVelocity(getTargetRPM() / Settings.SECONDS_IN_A_MINUTE));
            }
        } else {
            motor.stopMotor();
        }
        
        SmartDashboard.putBoolean("Robot/CAN/Main/Handoff Motor Connected? (ID " + String.valueOf(motor.getDeviceID()) + ")", motor.isConnected());

        if (Settings.DEBUG_MODE) {
            SmartDashboard.putNumber("Handoff/Voltage", motor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Handoff/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Handoff/Stator Current", motor.getStatorCurrent().getValueAsDouble());
            
            SmartDashboard.putNumber("Current Draws/Handoff (amps)", motor.getSupplyCurrent().getValueAsDouble());
        }
    }
    
    @Override
    public void setVoltageOverride(Optional<Double> voltage) {
        this.voltageOverride = voltage;
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
    
    @Override
    public double getCurrentDraw(){
        return motor.getSupplyCurrent().getValueAsDouble();
    }
}