/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.spindexer;

import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.robot.Robot;
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

    private final TalonFX leaderMotor;
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

        leaderMotor = new TalonFX(Ports.Spindexer.SPINDEXER_LEAD_MOTOR, Ports.CANIVORE);
        followerMotor = new TalonFX(Ports.Spindexer.SPINDEXER_FOLLOW_MOTOR, Ports.CANIVORE);

        spindexerLeadConfig.configure(leaderMotor);
        spindexerFollowerConfig.configure(followerMotor);

        controller = new VelocityVoltage(getTargetRPM()).withEnableFOC(true);
        follower = new Follower(Ports.Spindexer.SPINDEXER_LEAD_MOTOR, MotorAlignmentValue.Aligned);

        isStalling = BStream.create(() -> leaderMotor.getSupplyCurrent().getValueAsDouble() > Settings.Spindexer.STALL_CURRENT_LIMIT)
                .filtered(new BDebounce.Both(Settings.Superstructure.Hood.STALL_DEBOUNCE));

        followerMotor.setControl(follower);

        voltageOverride = Optional.empty();
    }

    private double getCurrentLeaderMotorRPM() {
        return leaderMotor.getVelocity().getValueAsDouble() * Settings.SECONDS_IN_A_MINUTE * Settings.Spindexer.GEAR_RATIO;
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
        double error = getCurrentLeaderMotorRPM() - getTargetRPM();
        return Math.abs(error) <= Settings.Spindexer.RPM_TOLERANCE;
    }

    @Override
    public boolean canStartIntakeRollers() {
        double error = getCurrentLeaderMotorRPM() - getTargetRPM();
        return Math.abs(error) <= Settings.Spindexer.TOLERANCE_TO_START_INTAKE_ROLLERS_DURING_SCORING_ROUTINE;
    }

    @Override
    public void periodic() {
        super.periodic();

        if (EnabledSubsystems.SPINDEXER.get()) {
            if (voltageOverride.isPresent()) {
                leaderMotor.setVoltage(voltageOverride.get());
            } else {
                if (shouldStop()) {
                    leaderMotor.stopMotor();
                } else {
                    leaderMotor.setControl(controller.withVelocity(getTargetRPM() / Settings.SECONDS_IN_A_MINUTE));
                }
            }
        } else {
            leaderMotor.stopMotor();
        }

        SmartDashboard.putNumber("Spindexer/Leader Motor RPM", getCurrentLeaderMotorRPM());
        SmartDashboard.putNumber("Spindexer/Follower Motor RPM", getCurrentFollowerMotorRPM());

        SmartDashboard.putBoolean("Spindexer/At Tolerance", atTolerance());

        SmartDashboard.putNumber("Spindexer/Leader Voltage (volts)", leaderMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Spindexer/Leader Supply Current (amps)", leaderMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Spindexer/Leader Stator Current (amps)", leaderMotor.getStatorCurrent().getValueAsDouble());



        if (Settings.DEBUG_MODE.get()) {
            SmartDashboard.putBoolean("Robot/CAN/Main/Spindexer Leader Motor Connected? (ID " + String.valueOf(leaderMotor.getDeviceID()) + ")", leaderMotor.isConnected());
            SmartDashboard.putBoolean("Robot/CAN/Main/Spindexer Follower Motor Connected? (ID " + String.valueOf(followerMotor.getDeviceID()) + ")", followerMotor.isConnected());
            SmartDashboard.putNumber("Spindexer/Follower Voltage (volts)", followerMotor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Spindexer/Follower Supply Current (amps)", followerMotor.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Spindexer/Follower Stator Current (amps)", followerMotor.getStatorCurrent().getValueAsDouble());
            SmartDashboard.putBoolean("Spindexer/Should Stop?", shouldStop());
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
            () -> leaderMotor.getPosition().getValueAsDouble(),
            () -> leaderMotor.getVelocity().getValueAsDouble(),
            () -> leaderMotor.getMotorVoltage().getValueAsDouble(),
            getInstance()
        );
    }

    @Override
    public double getCurrentDraw() {
        return  Math.abs(leaderMotor.getSupplyCurrent().getValueAsDouble()) + 
                Math.abs(followerMotor.getSupplyCurrent().getValueAsDouble());
    }
}