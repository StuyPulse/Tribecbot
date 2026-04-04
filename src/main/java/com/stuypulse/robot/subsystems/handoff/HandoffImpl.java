/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.handoff;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.RobotMode;
import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.util.PhoenixUtil;
import com.stuypulse.robot.util.SysId;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class HandoffImpl extends Handoff {
    private final Motors.TalonFXConfig handoffConfig;

    private final TalonFX motorLead;
    private final TalonFX motorFollow;
    // private final VelocityVoltage controller;
    private final DutyCycleOut controller;

    private Optional<Double> voltageOverride;
    private BStream isStalling;
    private final Follower follower;

    private StatusSignal<Current> motorLeadSupplyCurrent;
    private StatusSignal<Current> motorLeadStatorCurrent;
    private StatusSignal<AngularVelocity> motorLeadVelocity;
    private StatusSignal<Voltage> motorLeadVoltage;

    private StatusSignal<Current> motorFollowSupplyCurrent;
    private StatusSignal<Current> motorFollowStatorCurrent;
    private StatusSignal<AngularVelocity> motorFollowVelocity;
    private StatusSignal<Voltage> motorFollowVoltage;
    BaseStatusSignal[] signals;

    public HandoffImpl() {
        handoffConfig = new Motors.TalonFXConfig()
            .withInvertedValue(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
            
            .withSupplyCurrentLimitAmps(80.0)
            .withStatorCurrentLimitEnabled(false)
            .withRampRate(0.25)
            
            .withPIDConstants(Gains.Handoff.kP, Gains.Handoff.kI, Gains.Handoff.kD, 0)
            .withFFConstants(Gains.Handoff.kS, Gains.Handoff.kV, Gains.Handoff.kA, 0)
            
            .withSensorToMechanismRatio(Settings.Handoff.GEAR_RATIO);

        motorLead = new TalonFX(Ports.Handoff.MOTOR_LEAD, Ports.RIO);
        motorFollow = new TalonFX(Ports.Handoff.MOTOR_FOLLOW, Ports.RIO);
        handoffConfig.configure(motorLead);
        handoffConfig.configure(motorFollow);

        // controller = new VelocityVoltage(getTargetRPM() / Settings.SECONDS_IN_A_MINUTE).withEnableFOC(true);
        controller = new DutyCycleOut(getTargetDutyCycle());
        follower = new Follower(Ports.Handoff.MOTOR_LEAD, MotorAlignmentValue.Opposed);
        voltageOverride = Optional.empty();

        motorLeadSupplyCurrent = motorLead.getSupplyCurrent();
        motorLeadStatorCurrent = motorLead.getStatorCurrent();
        motorLeadVelocity = motorLead.getVelocity();
        motorLeadVoltage = motorLead.getMotorVoltage();
        motorFollowSupplyCurrent = motorFollow.getSupplyCurrent();
        motorFollowStatorCurrent = motorFollow.getStatorCurrent();
        motorFollowVelocity = motorFollow.getVelocity();
        motorFollowVoltage = motorFollow.getMotorVoltage();
        signals = new BaseStatusSignal[]{motorLeadSupplyCurrent, motorLeadStatorCurrent, motorLeadVelocity, motorLeadVoltage, motorFollowSupplyCurrent, motorFollowStatorCurrent, motorFollowVelocity, motorFollowVoltage};
        PhoenixUtil.registerToRio(signals);

        isStalling = BStream.create(() -> motorLeadSupplyCurrent.getValueAsDouble() > Settings.Handoff.HANDOFF_STALL_CURRENT.getAsDouble())
            .filtered(new BDebounce.Both(Settings.Handoff.HANDOFF_STALL_DEBOUNCE_SEC));
    }

    @Override
    public boolean isHandoffStalling() {
        return isStalling.get();
    }

    public double getLeaderRPM() {
        return motorLeadVelocity.getValueAsDouble() * Settings.SECONDS_IN_A_MINUTE;
    }

    public double getFollowerRPM() {
        return motorFollowVelocity.getValueAsDouble() * Settings.SECONDS_IN_A_MINUTE;
    }

    
    
    @Override
    public void periodicAfterScheduler() {
        super.periodicAfterScheduler();

        // removed shouldNotShootIntoHub logic (no longer used)
        
        if (EnabledSubsystems.HANDOFF.get() && getState() != HandoffState.STOP) {
            if (voltageOverride.isPresent()) {
                motorLead.setVoltage(voltageOverride.get());
            } else if (Superstructure.getInstance().shouldStop()) {
                motorLead.stopMotor();
                motorFollow.stopMotor();
            } else {
                motorLead.setControl(controller.withOutput(getTargetDutyCycle()));
                motorFollow.setControl(follower);
            }
        } else {
            motorLead.stopMotor();
            motorFollow.stopMotor();
        }
        
        SmartDashboard.putBoolean("Handoff/Should Stop?", Superstructure.getInstance().shouldStop());
        SmartDashboard.putNumber("Handoff/Lead Velocity", getLeaderRPM());
        SmartDashboard.putNumber("Handoff/Follow Velocity", getLeaderRPM());
        // SmartDashboard.putBoolean("Spindexer/Should Stop", shouldStop()); logged twice
        
        DogLog.log("Handoff/Lead Voltage", motorLeadVoltage.getValueAsDouble());
        DogLog.log("Handoff/Lead Supply Current", motorLeadSupplyCurrent.getValueAsDouble());
        DogLog.log("Handoff/Lead Stator Current", motorLeadStatorCurrent.getValueAsDouble());
        DogLog.log("Handoff/Follow Voltage", motorLeadVoltage.getValueAsDouble());
        DogLog.log("Handoff/Follow Supply Current", motorLeadSupplyCurrent.getValueAsDouble());
        DogLog.log("Handoff/Follow Stator Current", motorLeadStatorCurrent.getValueAsDouble());


        if (Settings.DEBUG_MODE.get()) {     
            SmartDashboard.putNumber("Handoff/Lead Voltage", motorLeadVoltage.getValueAsDouble());
            SmartDashboard.putNumber("Handoff/Lead Supply Current", motorLeadSupplyCurrent.getValueAsDouble());
            SmartDashboard.putNumber("Handoff/Lead Stator Current", motorLeadStatorCurrent.getValueAsDouble());

            SmartDashboard.putNumber("Handoff/Follow Voltage", motorLeadVoltage.getValueAsDouble());
            SmartDashboard.putNumber("Handoff/Follow Supply Current", motorLeadSupplyCurrent.getValueAsDouble());
            SmartDashboard.putNumber("Handoff/Follow Stator Current", motorLeadStatorCurrent.getValueAsDouble());
            
            if(Robot.getMode() == RobotMode.DISABLED && !DriverStation.isFMSAttached()) {
                SmartDashboard.putBoolean("Robot/CAN/Main/Handoff Lead Motor Connected? (ID " + String.valueOf(Ports.Handoff.MOTOR_LEAD) + ")", motorLead.isConnected());
                SmartDashboard.putBoolean("Robot/CAN/Main/Handoff Follow Motor Connected? (ID " + String.valueOf(Ports.Handoff.MOTOR_FOLLOW) + ")", motorFollow.isConnected());
            }
        }

        Robot.getEnergyUtil().logEnergyUsage(getName(), getCurrentDraw());
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
            () -> motorLead.getPosition().getValueAsDouble(),
            () -> motorLead.getVelocity().getValueAsDouble(),
            () -> motorLead.getMotorVoltage().getValueAsDouble(),
            getInstance());
    }
    
    @Override
    public double getCurrentDraw(){
        return  Double.max(0, motorLeadSupplyCurrent.getValueAsDouble()) + 
                Double.max(0, motorFollowSupplyCurrent.getValueAsDouble());
    }
}