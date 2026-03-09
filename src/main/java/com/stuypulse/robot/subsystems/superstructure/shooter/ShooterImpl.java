/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.superstructure.shooter;

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
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import java.util.Optional;

public class ShooterImpl extends Shooter {
    private final Motors.TalonFXConfig shooterConfig;

    private final TalonFX shooterLeader;
    private final TalonFX shooterFollower;

    private final VelocityVoltage shooterController;
    private final Follower follower;

    private Optional<Double> voltageOverride;

    public ShooterImpl() {
        shooterConfig = new Motors.TalonFXConfig()
            .withInvertedValue(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast)
            
            .withSupplyCurrentLimitEnabled(false)
            .withStatorCurrentLimitEnabled(false)

            .withPIDConstants(Gains.Superstructure.Shooter.kP.get(), Gains.Superstructure.Shooter.kI.get(), Gains.Superstructure.Shooter.kD.get(), 0)
            .withFFConstants(Gains.Superstructure.Shooter.kS.get(), Gains.Superstructure.Shooter.kV.get(), Gains.Superstructure.Shooter.kA.get(), 0)
                             
            .withSensorToMechanismRatio(Settings.Superstructure.Shooter.GEAR_RATIO)
            .withVelocityTimeFilter(0.1);

        shooterLeader = new TalonFX(Ports.Superstructure.Shooter.MOTOR_LEAD, Ports.RIO);
        shooterLeader.getVelocity().setUpdateFrequency(1000.0);

        shooterFollower = new TalonFX(Ports.Superstructure.Shooter.MOTOR_FOLLOW, Ports.RIO);
        shooterFollower.getVelocity().setUpdateFrequency(1000.0);

        shooterConfig.configure(shooterLeader);
        shooterConfig.configure(shooterFollower);

        shooterController = new VelocityVoltage(getTargetRPM() / Settings.SECONDS_IN_A_MINUTE).withEnableFOC(true);
        follower = new Follower(Ports.Superstructure.Shooter.MOTOR_LEAD, MotorAlignmentValue.Opposed);

        shooterFollower.setControl(follower);

        voltageOverride = Optional.empty();
    }

    @Override
    public double getRPM() {
        return getLeaderRPM();
    }

    public double getLeaderRPM() {
        return shooterLeader.getVelocity().getValueAsDouble() * Settings.SECONDS_IN_A_MINUTE;
    }

    public double getFollowerRPM() {
        return shooterFollower.getVelocity().getValueAsDouble() * Settings.SECONDS_IN_A_MINUTE;
    }

    @Override
    public void periodic() {
        super.periodic();

        shooterConfig.updateGainsConfig(
            shooterLeader,
            0,
            Gains.Superstructure.Shooter.kP,
            Gains.Superstructure.Shooter.kI,
            Gains.Superstructure.Shooter.kD,
            Gains.Superstructure.Shooter.kS,
            Gains.Superstructure.Shooter.kV,
            Gains.Superstructure.Shooter.kA
        );

        shooterConfig.updateGainsConfig(
            shooterFollower,
            0,
            Gains.Superstructure.Shooter.kP,
            Gains.Superstructure.Shooter.kI,
            Gains.Superstructure.Shooter.kD,
            Gains.Superstructure.Shooter.kS,
            Gains.Superstructure.Shooter.kV,
            Gains.Superstructure.Shooter.kA
        );

        if (EnabledSubsystems.SHOOTER.get()) {
            if (getState() == ShooterState.STOP) {
                shooterLeader.stopMotor();
            } else if (voltageOverride.isPresent()) {
                shooterLeader.setVoltage(voltageOverride.get());
                shooterFollower.setControl(follower);
            } else {
                shooterLeader.setControl(shooterController.withVelocity(getTargetRPM() / Settings.SECONDS_IN_A_MINUTE));
                shooterFollower.setControl(follower);
            }
        } else {
            shooterLeader.stopMotor();
        }

        if (Settings.DEBUG_MODE) {
            SmartDashboard.putNumber("Superstructure/Shooter/Leader Current (amps)", shooterLeader.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Superstructure/Shooter/Follower Supply Current (amps)", shooterFollower.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Superstructure/Shooter/Follower Stator Current", shooterFollower.getStatorCurrent().getValueAsDouble());

            SmartDashboard.putNumber("Superstructure/Shooter/Leader Voltage", shooterLeader.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Superstructure/Shooter/Follower Voltage", shooterFollower.getMotorVoltage().getValueAsDouble());

            SmartDashboard.putNumber("Superstructure/Shooter/Follower RPM", getFollowerRPM());

            SmartDashboard.putNumber("InterpolationTesting/Shooter Closed Loop Error", shooterLeader.getClosedLoopError().getValueAsDouble() * 60.0);
            SmartDashboard.putNumber("InterpolationTesting/Shooter Applied Voltage", shooterLeader.getMotorVoltage().getValueAsDouble());

            SmartDashboard.putNumber("Current Draws/Shooter Leader (amps)", shooterLeader.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Current Draws/Shooter Follower (amps)", shooterFollower.getSupplyCurrent().getValueAsDouble());
        }
    }

    private void setVoltageOverride(Optional<Double> voltageOverride) {
        this.voltageOverride = voltageOverride;
    }

    @Override
    public SysIdRoutine getShooterSysIdRoutine() {
        return SysId.getRoutine(
            1,
            5,
            "Shooter",
            voltage -> setVoltageOverride(Optional.of(voltage)),
            () -> shooterLeader.getPosition().getValueAsDouble(),
            () -> shooterLeader.getVelocity().getValueAsDouble(),
            () -> shooterLeader.getMotorVoltage().getValueAsDouble(),
            getInstance()
        );
    }
}