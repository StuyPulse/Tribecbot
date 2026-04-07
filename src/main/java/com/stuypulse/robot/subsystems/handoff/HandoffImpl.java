/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.handoff;

import java.util.Optional;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.util.MasterLogger;
import com.stuypulse.robot.util.MotorLogger;
import com.stuypulse.robot.util.MotorLogger.SubsystemName;
import com.stuypulse.robot.util.MotorLogger.ValueKey;
import com.stuypulse.robot.util.PhoenixUtil.PublishingDestination;
import com.stuypulse.robot.util.SysId;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

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

    private final MotorLogger leadLogger;
    private final MotorLogger followLogger;
    private final MasterLogger handOffLogger;

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

        leadLogger  = new MotorLogger(SubsystemName.Handoff, "Lead Motor", motorLead, Ports.Handoff.MOTOR_LEAD, PublishingDestination.RIO);
        followLogger  = new MotorLogger(SubsystemName.Handoff, "Follower Motor", motorFollow, Ports.Handoff.MOTOR_FOLLOW, PublishingDestination.RIO);
        handOffLogger = new MasterLogger(leadLogger, followLogger);

        // controller = new VelocityVoltage(getTargetRPM() / Settings.SECONDS_IN_A_MINUTE).withEnableFOC(true);
        controller = new DutyCycleOut(getTargetDutyCycle());
        follower = new Follower(Ports.Handoff.MOTOR_LEAD, MotorAlignmentValue.Opposed);
        voltageOverride = Optional.empty();
        isStalling = BStream.create(() -> handOffLogger.getMotorSignalMap(leadLogger).get(ValueKey.Supply_Current.toString()).getValueAsDouble() > Settings.Handoff.HANDOFF_STALL_CURRENT.getAsDouble())
            .filtered(new BDebounce.Both(Settings.Handoff.HANDOFF_STALL_DEBOUNCE_SEC));
    }

    @Override
    public boolean isHandoffStalling() {
        return isStalling.get();
    }

    public double getLeaderRPM() {
        return handOffLogger.getMotorSignalMap(leadLogger).get(ValueKey.VelocityRPS.toString()).getValueAsDouble() * Settings.SECONDS_IN_A_MINUTE;
    }

    public double getFollowerRPM() {
        return handOffLogger.getMotorSignalMap(followLogger).get(ValueKey.VelocityRPS.toString()).getValueAsDouble() * Settings.SECONDS_IN_A_MINUTE;
        //now already logged
    }

    @Override
    public void periodicAfterScheduler() {
        handOffLogger.logEverything();

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
        return  Double.max(0, handOffLogger.getMotorSignalMap(leadLogger).get(ValueKey.Supply_Current.toString()).getValueAsDouble()) + 
                Double.max(0, handOffLogger.getMotorSignalMap(followLogger).get(ValueKey.Supply_Current.toString()).getValueAsDouble());
    }
}