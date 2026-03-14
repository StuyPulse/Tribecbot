/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.spindexer;

import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.handoff.Handoff.HandoffState;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.SysId;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import java.util.Optional;

public class SpindexerImpl extends Spindexer {
    private final Motors.TalonFXConfig spindexerLeadConfig;
    private final Motors.TalonFXConfig spindexerFollowerConfig;

    private final TalonFX leadMotor;
    private final TalonFX followerMotor;

    private final VelocityVoltage controller;
    private final Follower follower;
    private final BStream isStalling;

    private Optional<Double> voltageOverride;

    public SpindexerImpl() {
        spindexerLeadConfig = new Motors.TalonFXConfig()
            .withInvertedValue(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
            
            .withSupplyCurrentLimitAmps(45)
            .withStatorCurrentLimitEnabled(false)
            .withRampRate(0.25)
            
            .withPIDConstants(Gains.Spindexer.kP, Gains.Spindexer.kI, Gains.Spindexer.kD, 0)
            .withFFConstants(Gains.Spindexer.kS, Gains.Spindexer.kV, Gains.Spindexer.kA, 0)
            
            .withSensorToMechanismRatio(Settings.Spindexer.GEAR_RATIO);

        spindexerFollowerConfig = new Motors.TalonFXConfig()
            .withInvertedValue(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
            
            .withSupplyCurrentLimitAmps(45)
            .withStatorCurrentLimitEnabled(false)
            .withRampRate(0.25)
            
            .withFFConstants(Gains.Spindexer.kS, Gains.Spindexer.kV, Gains.Spindexer.kA, 0)
            .withPIDConstants(Gains.Spindexer.kP, Gains.Spindexer.kI, Gains.Spindexer.kD, 0)
            
            .withSensorToMechanismRatio(Settings.Spindexer.GEAR_RATIO);

        leadMotor = new TalonFX(Ports.Spindexer.SPINDEXER_LEAD_MOTOR, Ports.CANIVORE);
        followerMotor = new TalonFX(Ports.Spindexer.SPINDEXER_FOLLOW_MOTOR, Ports.CANIVORE);

        spindexerLeadConfig.configure(leadMotor);
        spindexerFollowerConfig.configure(followerMotor);

        controller = new VelocityVoltage(getTargetRPM()).withEnableFOC(true);
        follower = new Follower(Ports.Spindexer.SPINDEXER_LEAD_MOTOR, MotorAlignmentValue.Aligned);

        isStalling = BStream.create(() -> leadMotor.getSupplyCurrent().getValueAsDouble() > Settings.Spindexer.STALL_CURRENT_LIMIT)
                .filtered(new BDebounce.Both(Settings.Superstructure.Hood.STALL_DEBOUNCE));

        followerMotor.setControl(follower);

        voltageOverride = Optional.empty();
    }

    private double getCurrentLeadMotorRPM() {
        return leadMotor.getVelocity().getValueAsDouble() * Settings.SECONDS_IN_A_MINUTE * Settings.Spindexer.GEAR_RATIO;
    }

    private double getCurrentFollowerMotorRPM() {
        return followerMotor.getVelocity().getValueAsDouble() * Settings.SECONDS_IN_A_MINUTE * Settings.Spindexer.GEAR_RATIO;
    }

    public boolean shouldStop() {
        boolean isStopState = getState() == SpindexerState.STOP;
        boolean isTurretWrapping = Superstructure.getInstance().isTurretWrapping();
        boolean isBehindHubWhileFerrying = Superstructure.getInstance().getState() == SuperstructureState.FOTM && CommandSwerveDrivetrain.getInstance().isBehindHub();

        return isStopState || isTurretWrapping || isBehindHubWhileFerrying;
    }

    @Override
    public boolean atTolerance() {
        double error = getCurrentLeadMotorRPM() - getTargetRPM();
        return Math.abs(error) <= Settings.Spindexer.RPM_TOLERANCE;
    }

    @Override
    public boolean canStartIntakeRollers() {
        double error = getCurrentLeadMotorRPM() - getTargetRPM();
        return Math.abs(error) <= Settings.Spindexer.TOLERANCE_TO_START_INTAKE_ROLLERS_DURING_SCORING_ROUTINE;
    }

    @Override
    public void periodic() {
        super.periodic();

        if (EnabledSubsystems.SPINDEXER.get()) {
            if (voltageOverride.isPresent()) {
                leadMotor.setVoltage(voltageOverride.get());
            } else {
                if (shouldStop()) {
                    leadMotor.stopMotor();
                } else {
                    leadMotor.setControl(controller.withVelocity(getTargetRPM() / Settings.SECONDS_IN_A_MINUTE));
                }
            }
        } else {
            leadMotor.stopMotor();
        }

        SmartDashboard.putBoolean("Robot/CAN/Main/Spindexer Leader Motor Connected? (ID " + String.valueOf(leadMotor.getDeviceID()) + ")", leadMotor.isConnected());
        SmartDashboard.putBoolean("Robot/CAN/Main/Spindexer Follower Motor Connected? (ID " + String.valueOf(followerMotor.getDeviceID()) + ")", followerMotor.isConnected());

        if (Settings.DEBUG_MODE) {
            SmartDashboard.putNumber("Spindexer/Lead Motor RPM", getCurrentLeadMotorRPM());
            SmartDashboard.putNumber("Spindexer/Follower Motor RPM", getCurrentFollowerMotorRPM());

            SmartDashboard.putBoolean("Spindexer/At Tolerance", atTolerance());

            SmartDashboard.putNumber("Spindexer/Lead Voltage", leadMotor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Spindexer/Lead Stator Current", leadMotor.getStatorCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Spindexer/Lead Supply Current", leadMotor.getSupplyCurrent().getValueAsDouble());

            SmartDashboard.putNumber("Spindexer/Follower Voltage", followerMotor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Spindexer/Follower Supply Current", followerMotor.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Spindexer/Follower Stator Current", followerMotor.getStatorCurrent().getValueAsDouble());

            SmartDashboard.putNumber("Current Draws/Spindexer Leader (amps)", leadMotor.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Current Draws/Spindexer Follower (amps)", followerMotor.getSupplyCurrent().getValueAsDouble());
        }
    }

    public boolean isStalling() {
        return isStalling.get();
    }

    @Override
    public void setVoltageOverride(Optional<Double> voltage) {
        this.voltageOverride = voltage;
    }

    @Override
    public SysIdRoutine getSysIdRoutine() {
        return SysId.getRoutine(
            1,
            2,
            "Spindexer",
            voltage -> setVoltageOverride(Optional.of(voltage)),
            () -> leadMotor.getPosition().getValueAsDouble(),
            () -> leadMotor.getVelocity().getValueAsDouble(),
            () -> leadMotor.getMotorVoltage().getValueAsDouble(),
            getInstance()
        );
    }

    @Override
    public double getCurrentDraw() {
        return  leadMotor.getSupplyCurrent().getValueAsDouble() + 
                followerMotor.getSupplyCurrent().getValueAsDouble();
    }
}