/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.hoodedshooter.shooter;

import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
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

public class ShooterImpl extends Shooter {
    private final TalonFX shooterLeader;
    private final TalonFX shooterFollower;

    private final VelocityVoltage shooterController;
    private final Follower follower;

    private Optional<Double> voltageOverride;

    public ShooterImpl() {
        shooterLeader = new TalonFX(Ports.HoodedShooter.Shooter.MOTOR_LEAD);
        shooterFollower = new TalonFX(Ports.HoodedShooter.Shooter.MOTOR_FOLLOW);

        shooterController = new VelocityVoltage(getTargetRPM() / 60.0);
        follower = new Follower(Ports.HoodedShooter.Shooter.MOTOR_LEAD, MotorAlignmentValue.Opposed);

        Motors.HoodedShooter.Shooter.SHOOTER.configure(shooterLeader);
        Motors.HoodedShooter.Shooter.SHOOTER.configure(shooterFollower);

        shooterFollower.setControl(follower);

        voltageOverride = Optional.empty();
    }

    @Override
    public double getShooterRPM() {
        return getLeaderRPM();
    }

    public double getLeaderRPM() {
        return shooterLeader.getVelocity().getValueAsDouble() * 60.0;
    }

    public double getFollowerRPM() {
        return shooterFollower.getVelocity().getValueAsDouble() * 60.0;
    }

    @Override 
    public void periodic() {
        super.periodic();

        if (EnabledSubsystems.SHOOTER.get()) {
            if (getState() == ShooterState.STOP) {
                shooterLeader.stopMotor();
                shooterFollower.stopMotor();
            } else if (voltageOverride.isPresent()) {
                shooterLeader.setVoltage(voltageOverride.get());
                shooterFollower.setControl(follower);
            } else {
                shooterLeader.setControl(shooterController.withVelocity(getTargetRPM() / 60.0).withEnableFOC(true));
                shooterFollower.setControl(follower);
            }
        } else {
            shooterLeader.stopMotor();
            shooterFollower.stopMotor();
        }
        
        if (Settings.DEBUG_MODE) {
            SmartDashboard.putNumber("HoodedShooter/Shooter/Leader Current (amps)", shooterLeader.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("HoodedShooter/Shooter/Follower Current (amps)", shooterFollower.getSupplyCurrent().getValueAsDouble());

            SmartDashboard.putNumber("HoodedShooter/Shooter/Leader Voltage", shooterLeader.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("HoodedShooter/Shooter/Follower Voltage", shooterFollower.getMotorVoltage().getValueAsDouble());

            SmartDashboard.putNumber("HoodedShooter/Shooter/Follower RPM", getFollowerRPM());

            SmartDashboard.putNumber("InterpolationTesting/Shooter Closed Loop Error", shooterLeader.getClosedLoopError().getValueAsDouble() * 60.0);
            SmartDashboard.putNumber("InterpolationTesting/Shooter Applied Voltage", shooterLeader.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("InterpolationTesting/Shooter Supply Current", shooterLeader.getSupplyCurrent().getValueAsDouble());
        }
    }

    public void setVoltageOverride(Optional<Double> voltageOverride) {
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