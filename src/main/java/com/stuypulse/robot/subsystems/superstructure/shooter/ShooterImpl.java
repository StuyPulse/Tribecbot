/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.superstructure.shooter;

import java.util.Optional;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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
import com.stuypulse.robot.util.MasterLogger;
import com.stuypulse.robot.util.MotorLogger;
import com.stuypulse.robot.util.MotorLogger.SubsystemName;
import com.stuypulse.robot.util.MotorLogger.ValueKey;
import com.stuypulse.robot.util.PhoenixUtil.PublishingDestination;
import com.stuypulse.robot.util.SysId;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ShooterImpl extends Shooter {
    private final Motors.TalonFXConfig shooterConfig;

    private final TalonFX shooterLeader;
    private final TalonFX shooterFollower;

    private final VelocityTorqueCurrentFOC shooterController;
    private final Follower follower;

    private Optional<Double> voltageOverride;

    private final MotorLogger shooterLeaderLogger;
    private final MotorLogger shooterFollowerLogger;
    private final MasterLogger shooterLogger;

    public ShooterImpl() {
        shooterConfig = new Motors.TalonFXConfig()
                .withInvertedValue(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast)

                .withSupplyCurrentLimitEnabled(false)
                .withStatorCurrentLimitEnabled(false)

                .withPIDConstants(Gains.Superstructure.Shooter.kP.get(), Gains.Superstructure.Shooter.kI.get(),
                        Gains.Superstructure.Shooter.kD.get(), 0)
                .withFFConstants(Gains.Superstructure.Shooter.kS.get(), Gains.Superstructure.Shooter.kV.get(),
                        Gains.Superstructure.Shooter.kA.get(), 0)

                .withSensorToMechanismRatio(Settings.Superstructure.Shooter.GEAR_RATIO)
                .withStatorCurrentLimitAmps(140)
                .withStatorCurrentLimitEnabled(false)
                .withSupplyCurrentLimitAmps(100)
                .withSupplyCurrentLimitEnabled(true)
                .withLowerLimitSupplyCurrent(60, 1);

        shooterLeader = new TalonFX(Ports.Superstructure.Shooter.MOTOR_LEAD, Ports.RIO);
        shooterLeader.getVelocity().setUpdateFrequency(1000.0);
        shooterLeader.getTorqueCurrent().setUpdateFrequency(1000.0);
        shooterLeader.getStatorCurrent().setUpdateFrequency(50.0);
        shooterLeader.getSupplyCurrent().setUpdateFrequency(50.0);

        shooterFollower = new TalonFX(Ports.Superstructure.Shooter.MOTOR_FOLLOW, Ports.RIO);
        shooterFollower.getVelocity().setUpdateFrequency(1000.0);
        shooterFollower.getTorqueCurrent().setUpdateFrequency(1000.0);
        shooterFollower.getStatorCurrent().setUpdateFrequency(50.0);
        shooterFollower.getSupplyCurrent().setUpdateFrequency(50.0);

        shooterConfig.configure(shooterLeader);
        shooterConfig.configure(shooterFollower);

        shooterController = new VelocityTorqueCurrentFOC(getTargetRPM() / Settings.SECONDS_IN_A_MINUTE);
        follower = new Follower(Ports.Superstructure.Shooter.MOTOR_LEAD, MotorAlignmentValue.Opposed);

        shooterFollower.setControl(follower);

        shooterLeaderLogger = new MotorLogger(SubsystemName.Shooter, "Leader Motor", shooterLeader, Ports.Superstructure.Shooter.MOTOR_LEAD, PublishingDestination.RIO);
        shooterFollowerLogger = new MotorLogger(SubsystemName.Shooter, "Leader Motor", shooterFollower, Ports.Superstructure.Shooter.MOTOR_FOLLOW, PublishingDestination.RIO);
        shooterLogger = new MasterLogger(shooterLeaderLogger, shooterFollowerLogger);

        voltageOverride = Optional.empty();
    }

    @Override
    public double getRPM() {
        return getLeaderRPM();
    }

    private double getLeaderRPM() {
        return shooterLogger.getMotorSignalMap(shooterLeaderLogger).get(ValueKey.VelocityRPS.toString()).getValueAsDouble() * Settings.SECONDS_IN_A_MINUTE;
    }

    private double getFollowerRPM() {
        return shooterLogger.getMotorSignalMap(shooterFollowerLogger).get(ValueKey.VelocityRPS.toString()).getValueAsDouble() * Settings.SECONDS_IN_A_MINUTE;
    }

    public double getBangBangOutput(double mesurement, double setpoint) {
        return mesurement < setpoint ? 1.0 : -1.0;
    }


    @Override
    public void periodicAfterScheduler() {
        shooterLogger.logEverything();

        super.periodicAfterScheduler();

        shooterConfig.updateGainsConfig(
                shooterLeader,
                0,
                Gains.Superstructure.Shooter.kP,
                Gains.Superstructure.Shooter.kI,
                Gains.Superstructure.Shooter.kD,
                Gains.Superstructure.Shooter.kS,
                Gains.Superstructure.Shooter.kV,
                Gains.Superstructure.Shooter.kA);

        shooterConfig.updateGainsConfig(
                shooterFollower,
                0,
                Gains.Superstructure.Shooter.kP,
                Gains.Superstructure.Shooter.kI,
                Gains.Superstructure.Shooter.kD,
                Gains.Superstructure.Shooter.kS,
                Gains.Superstructure.Shooter.kV,
                Gains.Superstructure.Shooter.kA);

        if (EnabledSubsystems.SHOOTER.get() || getState() == ShooterState.STOP) {
            if (voltageOverride.isPresent()) {
                shooterLeader.setVoltage(voltageOverride.get());
            } else {
                shooterLeader.setControl(shooterController.withVelocity(getTargetRPM() / Settings.SECONDS_IN_A_MINUTE));
            }
        } else {
            shooterLeader.stopMotor();
        }

        SmartDashboard.putNumber("Superstructure/Shooter/Leader RPM", getLeaderRPM());
        SmartDashboard.putNumber("Superstructure/Shooter/Follower RPM", getFollowerRPM());

        SmartDashboard.putNumber("InterpolationTesting/Shooter Closed Loop Error (RPM)",
                shooterLogger.getMotorSignalMap(shooterLeaderLogger).get(ValueKey.Closed_Loop_Error.toString()).getValueAsDouble() * 60.0);

        Robot.getEnergyUtil().logEnergyUsage(getName(), getCurrentDraw());
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
                getInstance());
    }

    @Override
    public double getCurrentDraw() {
        return Double.max(0, shooterLogger.getMotorSignalMap(shooterLeaderLogger).get(ValueKey.Supply_Current.toString()).getValueAsDouble()) +
                Double.max(0, shooterLogger.getMotorSignalMap(shooterFollowerLogger).get(ValueKey.Supply_Current.toString()).getValueAsDouble());
    }
}