/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.spindexer;

import java.util.Optional;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.handoff.Handoff;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.util.MasterLogger;
import com.stuypulse.robot.util.MotorLogger;
import com.stuypulse.robot.util.MotorLogger.ValueKey;
import com.stuypulse.robot.util.PhoenixUtil;
import com.stuypulse.robot.util.SysId;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SpindexerImpl extends Spindexer {
    private final Motors.TalonFXConfig spindexerLeadConfig;

    private final TalonFX leaderMotor;

    private final DutyCycleOut controller;
    private final BStream isStalling;
    private boolean hasStartedStallTimer;
    private final Timer unjamTimer;

    private Optional<Double> voltageOverride;

    private final MotorLogger spindexerLogger;
    private final MasterLogger spindexerMaster;

    public SpindexerImpl() {
        spindexerLeadConfig = new Motors.TalonFXConfig()
                .withInvertedValue(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)

                .withSupplyCurrentLimitAmps(45)
                .withStatorCurrentLimitEnabled(false)
                .withRampRate(0.25)

                .withPIDConstants(Gains.Spindexer.kP, Gains.Spindexer.kI, Gains.Spindexer.kD, 0)
                .withFFConstants(Gains.Spindexer.kS, Gains.Spindexer.kV, Gains.Spindexer.kA, 0)

                .withSensorToMechanismRatio(Settings.Spindexer.GEAR_RATIO);

        leaderMotor = new TalonFX(Ports.Spindexer.MOTOR, Ports.CANIVORE);

        spindexerLeadConfig.configure(leaderMotor);

        spindexerLogger = new MotorLogger(MotorLogger.SubsystemName.Spindexer, "Lead Motor", leaderMotor, Ports.Spindexer.MOTOR, PhoenixUtil.PublishingDestination.CANIVORE);
        spindexerMaster = new MasterLogger(spindexerLogger);

        controller = new DutyCycleOut(getTargetDutyCycle()).withEnableFOC(true);

        isStalling = BStream.create( () -> spindexerMaster.getMotorSignalMap(spindexerLogger).get(ValueKey.Supply_Current).getValueAsDouble() > Settings.Spindexer.STALL_CURRENT_LIMIT)
                .filtered(new BDebounce.Both(Settings.Superstructure.Hood.STALL_DEBOUNCE));
        voltageOverride = Optional.empty();

        hasStartedStallTimer = false;
        unjamTimer = new Timer();
    }

    private double getMotorRPM() {
        return spindexerMaster.getMotorSignalMap(spindexerLogger).get(ValueKey.VelocityRPS).getValueAsDouble() * Settings.SECONDS_IN_A_MINUTE * Settings.Spindexer.GEAR_RATIO;
    }

    private boolean spindexerUnjam() {
        if (!hasStartedStallTimer && Handoff.getInstance().isHandoffStalling()) {
            unjamTimer.start();
            hasStartedStallTimer = true;
            setState(SpindexerState.REVERSE);
            return true;
        } else if (unjamTimer.get() < Settings.Spindexer.REVERSE_TIME) {
            setState(SpindexerState.REVERSE);
            return true;
        } else {
            hasStartedStallTimer = false;
            return false;
        }
    }

    @Override
    public void periodicAfterScheduler() {
        spindexerMaster.logEverything();

        super.periodicAfterScheduler();

        // removed shouldNotShootIntoHub logic (no longer used)
    
        // boolean unJamming = spindexerUnjam();

        if (EnabledSubsystems.SPINDEXER.get()) {
            if (voltageOverride.isPresent()) {
                leaderMotor.setVoltage(voltageOverride.get());
            } else {
                if (Superstructure.getInstance().shouldStop()) {
                    leaderMotor.stopMotor();
                }             
                else {
                    leaderMotor.setControl(controller.withOutput(getTargetDutyCycle()));
                }
            }
        } else {
            leaderMotor.stopMotor();
        }

        // SmartDashboard.putBoolean("Spindexer/Unjamming", unJamming);
        SmartDashboard.putBoolean("Spindexer/Should Stop?", Superstructure.getInstance().shouldStop());

        Robot.getEnergyUtil().logEnergyUsage(getName(), getCurrentDraw());
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
                getInstance());
    }

    @Override
    public double getCurrentDraw() {
        return Double.max(0, spindexerMaster.getMotorSignalMap(spindexerLogger).get(ValueKey.Supply_Current).getValueAsDouble());
    }
}