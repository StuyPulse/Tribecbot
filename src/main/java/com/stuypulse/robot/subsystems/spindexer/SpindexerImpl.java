/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.spindexer;

import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.RobotMode;
import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.handoff.Handoff;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.SysId;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import java.lang.annotation.ElementType;
import java.util.Optional;

public class SpindexerImpl extends Spindexer {
    private final Motors.TalonFXConfig spindexerLeadConfig;
    private final Motors.TalonFXConfig spindexerFollowerConfig;

    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;

    private final VelocityVoltage controller;
    private final Follower follower;
    private final BStream isStalling;
    private boolean hasStartedStallTimer;
    private final Timer unjamTimer;

    private Optional<Double> voltageOverride;

    private StatusSignal<Current> leaderSupplyCurrent;
    private StatusSignal<Current> leaderStatorCurrent;
    private StatusSignal<Current> followerSupplyCurrent;
    private StatusSignal<Current> followerStatorCurrent;
    private StatusSignal<AngularVelocity> leaderVelocity;
    private StatusSignal<AngularVelocity> followerVelocity;
    private StatusSignal<Voltage> leaderMotorVoltage;
    private StatusSignal<Voltage> followerMotorVoltage;

    private BaseStatusSignal[] signals;

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

        followerMotor.setControl(follower);

        leaderSupplyCurrent = leaderMotor.getSupplyCurrent();
        leaderStatorCurrent = leaderMotor.getStatorCurrent();
        followerSupplyCurrent = followerMotor.getSupplyCurrent();
        followerStatorCurrent = followerMotor.getStatorCurrent();
        leaderVelocity = leaderMotor.getVelocity();
        followerVelocity = followerMotor.getVelocity();
        leaderMotorVoltage = leaderMotor.getMotorVoltage();
        followerMotorVoltage = followerMotor.getMotorVoltage();
        signals = new BaseStatusSignal[] { leaderSupplyCurrent, leaderStatorCurrent, followerSupplyCurrent,
                followerStatorCurrent, leaderVelocity, followerVelocity, leaderMotorVoltage, followerMotorVoltage };

        isStalling = BStream.create( () -> leaderSupplyCurrent.getValueAsDouble() > Settings.Spindexer.STALL_CURRENT_LIMIT)
                .filtered(new BDebounce.Both(Settings.Superstructure.Hood.STALL_DEBOUNCE));
        voltageOverride = Optional.empty();

        hasStartedStallTimer = false;
        unjamTimer = new Timer();
    }

    private double getCurrentLeaderMotorRPM() {
        return leaderVelocity.getValueAsDouble() * Settings.SECONDS_IN_A_MINUTE * Settings.Spindexer.GEAR_RATIO;
    }

    private double getCurrentFollowerMotorRPM() {
        return followerVelocity.getValueAsDouble() * Settings.SECONDS_IN_A_MINUTE * Settings.Spindexer.GEAR_RATIO;
    }

    public boolean shouldStop() {
        Superstructure superstructure = Superstructure.getInstance();
        SuperstructureState superstructureState = superstructure.getState();

        boolean isStopState = getState() == SpindexerState.STOP;
        boolean isTurretWrapping = superstructure.isTurretWrapping();
        boolean isBehindHubWhileFerrying = superstructureState == SuperstructureState.FOTM
                && CommandSwerveDrivetrain.getInstance().isBehindHub();
        boolean turretLaggingSOTM = !superstructure.isTurretAtTolerance() && superstructureState == SuperstructureState.SOTM;

        return isStopState || isTurretWrapping || isBehindHubWhileFerrying || turretLaggingSOTM;
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
    public void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(signals);
    }

    @Override
    public void periodic() {
        super.periodic();

        boolean shouldNotShootIntoHub = (Superstructure.getInstance().superstructureInShootIntoHubMode()) ? 
            !CommandSwerveDrivetrain.getInstance().canShootIntoHub() 
            : false;
    
        boolean unJamming = spindexerUnjam();
        setState(SpindexerState.REVERSE);

        if (EnabledSubsystems.SPINDEXER.get()) {
            if (voltageOverride.isPresent()) {
                leaderMotor.setVoltage(voltageOverride.get());
            } else {
                if ((shouldStop() || shouldNotShootIntoHub) && !unJamming) {
                    leaderMotor.stopMotor();
                }             
                else {
                    leaderMotor.setControl(controller.withVelocity(getTargetRPM() / Settings.SECONDS_IN_A_MINUTE));
                }
            }
        } else {
            leaderMotor.stopMotor();
        }

        SmartDashboard.putNumber("Spindexer/Leader Motor RPM", getCurrentLeaderMotorRPM());
        SmartDashboard.putNumber("Spindexer/Follower Motor RPM", getCurrentFollowerMotorRPM());
        SmartDashboard.putBoolean("Spindexer/Unjamming", unJamming);

        SmartDashboard.putBoolean("Spindexer/At Tolerance", atTolerance());

        SmartDashboard.putNumber("Spindexer/Leader Voltage (volts)", leaderMotorVoltage.getValueAsDouble());
        SmartDashboard.putNumber("Spindexer/Leader Supply Current (amps)", leaderSupplyCurrent.getValueAsDouble());
        SmartDashboard.putNumber("Spindexer/Leader Stator Current (amps)", leaderStatorCurrent.getValueAsDouble());

        SmartDashboard.putBoolean("Spindexer/Should Stop?", shouldStop());
        SmartDashboard.putBoolean("Spindexer/shouldNotShootIntoHub", shouldNotShootIntoHub);


        if (Settings.DEBUG_MODE.get()) {
            SmartDashboard.putNumber("Spindexer/Follower Voltage (volts)", followerMotorVoltage.getValueAsDouble());
            SmartDashboard.putNumber("Spindexer/Follower Supply Current (amps)", followerSupplyCurrent.getValueAsDouble());
            SmartDashboard.putNumber("Spindexer/Follower Stator Current (amps)", followerStatorCurrent.getValueAsDouble());

            if (Robot.getMode() == RobotMode.DISABLED && !DriverStation.isFMSAttached()) {
                SmartDashboard.putBoolean(
                        "Robot/CAN/Canivore/Spindexer Leader Motor Connected? (ID "
                                + String.valueOf(Ports.Spindexer.SPINDEXER_LEAD_MOTOR) + ")",
                        leaderMotor.isConnected());
                SmartDashboard.putBoolean(
                        "Robot/CAN/Canivore/Spindexer Follower Motor Connected? (ID "
                                + String.valueOf(Ports.Spindexer.SPINDEXER_FOLLOW_MOTOR) + ")",
                        followerMotor.isConnected());
            }
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
                getInstance());
    }

    @Override
    public double getCurrentDraw() {
        return Double.max(0, leaderSupplyCurrent.getValueAsDouble()) +
                Double.max(0, followerSupplyCurrent.getValueAsDouble());
    }
}