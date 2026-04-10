/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.spindexer;

import java.util.Optional;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.RobotMode;
import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.handoff.Handoff;
import com.stuypulse.robot.subsystems.handoff.Handoff.HandoffState;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.PhoenixUtil;
import com.stuypulse.robot.util.SysId;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import java.util.Optional;

public class SpindexerImpl extends Spindexer {
    private final Motors.TalonFXConfig spindexerLeadConfig;

    private final TalonFX leaderMotor;

    private final DutyCycleOut controller;
    private final BStream isStalling;
    private boolean hasStartedStallTimer;
    private final Timer unjamTimer;

    private Optional<Double> voltageOverride;

    private StatusSignal<Current> leaderSupplyCurrent;
    private StatusSignal<Current> leaderStatorCurrent;
    private StatusSignal<AngularVelocity> leaderVelocity;
    private StatusSignal<Voltage> leaderMotorVoltage;

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

        leaderMotor = new TalonFX(Ports.Spindexer.MOTOR, Ports.CANIVORE);

        spindexerLeadConfig.configure(leaderMotor);

        controller = new DutyCycleOut(getTargetDutyCycle()).withEnableFOC(true);

        leaderSupplyCurrent = leaderMotor.getSupplyCurrent();
        leaderStatorCurrent = leaderMotor.getStatorCurrent();
        leaderVelocity = leaderMotor.getVelocity();
        leaderMotorVoltage = leaderMotor.getMotorVoltage();
        PhoenixUtil.registerToCanivore(leaderSupplyCurrent, leaderStatorCurrent, leaderVelocity, leaderMotorVoltage);

        isStalling = BStream.create( () -> leaderSupplyCurrent.getValueAsDouble() > Settings.Spindexer.STALL_CURRENT_LIMIT)
                .filtered(new BDebounce.Both(Settings.Superstructure.Hood.STALL_DEBOUNCE));
        voltageOverride = Optional.empty();

        hasStartedStallTimer = false;
        unjamTimer = new Timer();
    }

    private double getMotorRPM() {
        return leaderVelocity.getValueAsDouble() * Settings.SECONDS_IN_A_MINUTE * Settings.Spindexer.GEAR_RATIO;
    }

    public boolean shouldStop() {
        Superstructure superstructure = Superstructure.getInstance();
        SuperstructureState superstructureState = superstructure.getState();
        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

        boolean isStopState = getState() == SpindexerState.STOP;
        boolean isTurretWrapping = superstructure.isTurretWrapping();
        boolean isBehindHubWhileFerrying = superstructureState == SuperstructureState.FOTM
                && swerve.isBehindHub();
        boolean isOutsideAllianceZone = 
            CommandSwerveDrivetrain.getInstance().isOutsideAllianceZone() && 
            superstructureState != SuperstructureState.FOTM;
        boolean isUnderTrench = CommandSwerveDrivetrain.getInstance().isUnderTrench() 
            && superstructureState != SuperstructureState.FOTM;
        boolean inManualState =       
            superstructureState == SuperstructureState.LEFT_CORNER &&
            superstructureState == SuperstructureState.RIGHT_CORNER &&
            superstructureState == SuperstructureState.KB;
        boolean isBehindTower = swerve.isBehindTower() && superstructureState == SuperstructureState.SOTM;

        boolean turretLaggingSOTM = !superstructure.isTurretAtTolerance() && superstructureState == SuperstructureState.SOTM;


        DogLog.log("Spindexer/Should Stop/turret lagging sotm", turretLaggingSOTM);
        DogLog.log("Spindexer/Should Stop/is Behind Hub While Ferrying?", isBehindHubWhileFerrying);
        DogLog.log("Spindexer/Should Stop/is Turret Wrapping?", isTurretWrapping);
        DogLog.log("Spindexer/Should Stop/is Outside Alliance zone?", isOutsideAllianceZone);
        DogLog.log("Spindexer/Should Stop/is Under Trenche?", isUnderTrench);
        DogLog.log("Spindexer/Should Stop/turret lagging sotm", turretLaggingSOTM);
        DogLog.log("Spindexer/Should Stop/inManualState", inManualState);
        return isStopState || 
        isTurretWrapping || 
        (isBehindHubWhileFerrying && !inManualState) || 
        turretLaggingSOTM || 
        (isOutsideAllianceZone  && !inManualState) || 
        (isUnderTrench && !inManualState) ||
        isBehindTower;
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
        super.periodicAfterScheduler();

        // removed shouldNotShootIntoHub logic (no longer used)
    
        // boolean unJamming = spindexerUnjam();

        if (EnabledSubsystems.SPINDEXER.get()) {
            if (voltageOverride.isPresent()) {
                leaderMotor.setVoltage(voltageOverride.get());
            } else {
                if (shouldStop()) {
                    leaderMotor.stopMotor();
                }             
                else {
                    leaderMotor.setControl(controller.withOutput(getTargetDutyCycle()));
                }
            }
        } else {
            leaderMotor.stopMotor();
        }

        DogLog.log("Spindexer/Leader Motor RPM", getMotorRPM());
        // SmartDashboard.putBoolean("Spindexer/Unjamming", unJamming);

        DogLog.log("Spindexer/Leader Voltage (volts)", leaderMotorVoltage.getValueAsDouble());
        DogLog.log("Spindexer/Leader Supply Current (amps)", leaderSupplyCurrent.getValueAsDouble());
        DogLog.log("Spindexer/Leader Stator Current (amps)", leaderStatorCurrent.getValueAsDouble());

    SmartDashboard.putBoolean("Spindexer/Should Stop?", shouldStop());

        if (Settings.DEBUG_MODE.get()) {
            if (Robot.getMode() == RobotMode.DISABLED && !DriverStation.isFMSAttached()) {
                SmartDashboard.putBoolean(
                        "Robot/CAN/Canivore/Spindexer Leader Motor Connected? (ID "
                                + String.valueOf(Ports.Spindexer.MOTOR) + ")",
                        leaderMotor.isConnected());
            }
            Robot.getEnergyUtil().logEnergyUsage(getName(), getCurrentDraw());
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
        return Double.max(0, leaderSupplyCurrent.getValueAsDouble());
    }
}