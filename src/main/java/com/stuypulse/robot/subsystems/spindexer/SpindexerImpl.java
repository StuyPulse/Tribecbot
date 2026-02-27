/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.spindexer;

import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.SysId;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import java.util.Optional;

public class SpindexerImpl extends Spindexer {
    private final TalonFX leadMotor;
    private final TalonFX followerMotor;

    private final VelocityVoltage controller;
    private final Follower follower;

    private Optional<Double> voltageOverride;

    public SpindexerImpl() {
        leadMotor = new TalonFX(Ports.Spindexer.SPINDEXER_LEAD_MOTOR, Ports.CANIVORE);
        followerMotor = new TalonFX(Ports.Spindexer.SPINDEXER_FOLLOW_MOTOR, Ports.CANIVORE);

        Motors.Spindexer.SPINDEXER_LEAD.configure(leadMotor);
        Motors.Spindexer.SPINDEXER_FOLLOWER.configure(followerMotor);

        controller = new VelocityVoltage(getTargetRPM())
            .withEnableFOC(true);

        follower = new Follower(Ports.Spindexer.SPINDEXER_LEAD_MOTOR, MotorAlignmentValue.Opposed);

        voltageOverride = Optional.empty();
    }

    public double getCurrentLeadMotorRPM() {
        return leadMotor.getVelocity().getValueAsDouble() * Settings.SECONDS_IN_A_MINUTE;
    }

    public double getCurrentFollowerMotorRPM() {
        return followerMotor.getVelocity().getValueAsDouble() * Settings.SECONDS_IN_A_MINUTE;
    }

    public boolean atTolerance() {
        return Math.abs(getCurrentLeadMotorRPM() - getTargetRPM()) <= Settings.Spindexer.RPM_TOLERANCE;
    }

    @Override
    public void periodic() {
        super.periodic();

        Motors.Spindexer.SPINDEXER_LEAD.updateGainsConfig(
            leadMotor, 
            0, 
            Gains.Spindexer.kP, 
            Gains.Spindexer.kI,
            Gains.Spindexer.kD,
            Gains.Spindexer.kS,
            Gains.Spindexer.kV,
            Gains.Spindexer.kA
        );

        Motors.Spindexer.SPINDEXER_FOLLOWER.updateGainsConfig(
            followerMotor, 
            0, 
            Gains.Spindexer.kP, 
            Gains.Spindexer.kI,
            Gains.Spindexer.kD,
            Gains.Spindexer.kS,
            Gains.Spindexer.kV,
            Gains.Spindexer.kA
        );

        if (EnabledSubsystems.SPINDEXER.get()) {
            if (voltageOverride.isPresent()){
                leadMotor.setVoltage(voltageOverride.get());
                followerMotor.setControl(follower);
            } else {
                leadMotor.setControl(controller.withVelocity(getTargetRPM() / Settings.SECONDS_IN_A_MINUTE));
                followerMotor.setControl(follower);
            }
        }

        if (Settings.DEBUG_MODE) {
            SmartDashboard.putNumber("Spindexer/Lead Motor RPM", getCurrentLeadMotorRPM());
            SmartDashboard.putNumber("Spindexer/Follower Motor RPM", getCurrentFollowerMotorRPM());

            SmartDashboard.putBoolean("Spindexer/At Tolerance", atTolerance());

            SmartDashboard.putNumber("Spindexer/Lead Current (amps)", leadMotor.getStatorCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Spindexer/Lead Voltage", leadMotor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Spindexer/Lead Supply Current", leadMotor.getSupplyCurrent().getValueAsDouble());

            SmartDashboard.putNumber("Spindexer/Follower Current (amps)", followerMotor.getStatorCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Spindexer/Follower Voltage", followerMotor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Spindexer/Follower Supply Current", followerMotor.getSupplyCurrent().getValueAsDouble());
        }
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
}