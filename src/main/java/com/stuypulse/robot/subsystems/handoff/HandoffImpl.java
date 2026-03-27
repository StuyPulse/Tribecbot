/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.handoff;

import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.RobotMode;
import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.spindexer.Spindexer.SpindexerState;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.FMSUtil;
import com.stuypulse.robot.util.PhoenixUtil;
import com.stuypulse.robot.util.SysId;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import java.util.Optional;

public class HandoffImpl extends Handoff {
    private final Motors.TalonFXConfig handoffConfig;

    private final TalonFX motor;
    // private final VelocityVoltage controller;
    private final DutyCycleOut controller;

    private Optional<Double> voltageOverride;
    private BStream isStalling;

    private StatusSignal<Current> motorSupplyCurrent;
    private StatusSignal<Current> motorStatorCurrent;
    private StatusSignal<AngularVelocity> motorVelocity;
    private StatusSignal<Voltage> motorVoltage;

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

        // controller = new VelocityVoltage(getTargetRPM() / Settings.SECONDS_IN_A_MINUTE).withEnableFOC(true);
        controller = new DutyCycleOut(getTargetDutyCycle());
        voltageOverride = Optional.empty();

        motorSupplyCurrent = motor.getSupplyCurrent();
        motorStatorCurrent = motor.getStatorCurrent();
        motorVelocity = motor.getVelocity();
        motorVoltage = motor.getMotorVoltage();
        PhoenixUtil.registerSignals(motorSupplyCurrent, motorStatorCurrent, motorVelocity, motorVoltage);

        isStalling = BStream.create(() -> motorSupplyCurrent.getValueAsDouble() > Settings.Handoff.HANDOFF_STALL_CURRENT.getAsDouble())
            .filtered(new BDebounce.Both(0.5));
    }

    @Override
    public boolean isHandoffStalling() {
        return isStalling.get();
    }

    public double getCurrentRPM() {
        return motorVelocity.getValueAsDouble() * Settings.SECONDS_IN_A_MINUTE;
    }

    public boolean shouldStop() {
        Superstructure superstructure = Superstructure.getInstance();
        SuperstructureState superstructureState = superstructure.getState();

        boolean isStopState = getState() == HandoffState.STOP;
        boolean isTurretWrapping = superstructure.isTurretWrapping();
        boolean isBehindHubWhileFerrying = superstructureState == SuperstructureState.FOTM
                && CommandSwerveDrivetrain.getInstance().isBehindHub();
        boolean isOutsideAllianceZone = 
            CommandSwerveDrivetrain.getInstance().isOutsideAllianceZone() && 
            superstructureState != superstructureState.FOTM;
        boolean isUnderTrench = CommandSwerveDrivetrain.getInstance().isUnderTrench() 
            && superstructureState != SuperstructureState.FOTM;
        boolean inManualState =       
            superstructureState == superstructureState.LEFT_CORNER &&
            superstructureState == superstructureState.RIGHT_CORNER &&
            superstructureState == superstructureState.KB;

        boolean turretLaggingSOTM = !superstructure.isTurretAtTolerance() && superstructureState == SuperstructureState.SOTM;

        return isStopState || 
        isTurretWrapping || 
        (isBehindHubWhileFerrying && !inManualState) || 
        turretLaggingSOTM || 
        (isOutsideAllianceZone  && !inManualState) || 
        (isUnderTrench && !inManualState);
    }
    
    @Override
    public void periodicAfterScheduler() {
        super.periodicAfterScheduler();

        boolean shouldNotShootIntoHub = (Superstructure.getInstance().superstructureInShootIntoHubMode()) ? 
            !CommandSwerveDrivetrain.getInstance().canShootIntoHub() 
            : false;
        
        if (EnabledSubsystems.HANDOFF.get() && getState() != HandoffState.STOP) {
            if (voltageOverride.isPresent()) {
                motor.setVoltage(voltageOverride.get());
            } else if (shouldStop() || shouldNotShootIntoHub) {
                motor.stopMotor();
            } else {
                motor.setControl(controller.withOutput(getTargetDutyCycle()));
            }
        } else {
            motor.stopMotor();
        }
        
        
        SmartDashboard.putBoolean("Handoff/ShouldStop?", shouldStop());
        SmartDashboard.putBoolean("Handoff/ShouldNotShootIntoHub", shouldNotShootIntoHub);
        SmartDashboard.putNumber("Handoff/Signal Velocity", motorVelocity.getValueAsDouble()  * Settings.SECONDS_IN_A_MINUTE);
        if (Settings.DEBUG_MODE.get()) {     
            SmartDashboard.putNumber("Handoff/Voltage", motorVoltage.getValueAsDouble());
            SmartDashboard.putNumber("Handoff/Supply Current", motorSupplyCurrent.getValueAsDouble());
            SmartDashboard.putNumber("Handoff/Stator Current", motorStatorCurrent.getValueAsDouble());
            
            if(Robot.getMode() == RobotMode.DISABLED && !DriverStation.isFMSAttached()) {
                SmartDashboard.putBoolean("Robot/CAN/Main/Handoff Motor Connected? (ID " + String.valueOf(Ports.Handoff.HANDOFF) + ")", motor.isConnected());
            }
        }

        Robot.getEnergyUtil().logEnergyUsage(getSubsystem(), getCurrentDraw());
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
        return Math.abs(motorSupplyCurrent.getValueAsDouble());
    }
}