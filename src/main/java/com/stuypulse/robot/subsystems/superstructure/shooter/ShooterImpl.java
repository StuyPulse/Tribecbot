/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.superstructure.shooter;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.RobotMode;
import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.PhoenixUtil;
import com.stuypulse.robot.util.SysId;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import java.util.Optional;

public class ShooterImpl extends Shooter {
    private final Motors.TalonFXConfig shooterConfig;

    private final TalonFX shooterLeader;
    private final TalonFX shooterFollower;

    private final VelocityTorqueCurrentFOC shooterController;
    private final Follower follower;

    private Optional<Double> voltageOverride;
    private StatusSignal<AngularVelocity> shooterLeaderSpeed;
    private StatusSignal<AngularVelocity> shooterFollowerSpeed;
    private StatusSignal<Current> shooterFollowSupplyCurrent;
    private StatusSignal<Current> shooterLeadSupplyCurrent;
    private StatusSignal<Current> shooterLeadStatorCurrent;
    private StatusSignal<Current> shooterFollowStatorCurrent;
    private StatusSignal<Voltage> shooterLeaderVoltage;
    private StatusSignal<Voltage> shooterFollowerVoltage;
    private StatusSignal<Double> shooterLeaderClosedLoopError;

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

        shooterLeaderSpeed = shooterLeader.getVelocity();
        shooterFollowerSpeed = shooterFollower.getVelocity();
        shooterFollowSupplyCurrent = shooterFollower.getSupplyCurrent();
        shooterFollowStatorCurrent = shooterFollower.getStatorCurrent();
        shooterLeadSupplyCurrent = shooterLeader.getSupplyCurrent();
        shooterLeadStatorCurrent = shooterLeader.getStatorCurrent();
        shooterLeaderVoltage = shooterLeader.getMotorVoltage();
        shooterFollowerVoltage = shooterLeader.getMotorVoltage();
        shooterLeaderClosedLoopError = shooterLeader.getClosedLoopError();
        PhoenixUtil.registerToRio(shooterLeaderSpeed, shooterFollowerSpeed, shooterFollowSupplyCurrent, 
                shooterFollowStatorCurrent, shooterLeadSupplyCurrent, shooterLeadStatorCurrent, 
                shooterLeaderVoltage, shooterFollowerVoltage, shooterLeaderClosedLoopError);
        voltageOverride = Optional.empty();
    }

    @Override
    public double getRPM() {
        return getLeaderRPM();
    }

    private double getLeaderRPM() {
        return shooterLeaderSpeed.getValueAsDouble() * Settings.SECONDS_IN_A_MINUTE;
    }

    private double getFollowerRPM() {
        return shooterFollowerSpeed.getValueAsDouble() * Settings.SECONDS_IN_A_MINUTE;
    }

    public double getBangBangOutput(double mesurement, double setpoint) {
        return mesurement < setpoint ? 1.0 : -1.0;
    }


    @Override
    public void periodicAfterScheduler() {
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

        DogLog.log("InterpolationTesting/Shooter Applied Voltage", shooterLeaderVoltage.getValueAsDouble());
        DogLog.log("Superstructure/Shooter/Leader Voltage (volts)", shooterLeaderVoltage.getValueAsDouble());
        DogLog.log("Superstructure/Shooter/Leader Supply Current (amps)", shooterLeadSupplyCurrent.getValueAsDouble());
        DogLog.log("Superstructure/Shooter/Leader Stator Current (amps)", shooterLeadStatorCurrent.getValueAsDouble());
        DogLog.log("Superstructure/Shooter/Follower Voltage (volts)", shooterFollowerVoltage.getValueAsDouble());
        DogLog.log("Superstructure/Shooter/Follower Supply Current (amps)", shooterFollowSupplyCurrent.getValueAsDouble());
        DogLog.log("Superstructure/Shooter/Follower Stator Current (amps)", shooterFollowStatorCurrent.getValueAsDouble());

        if (Settings.DEBUG_MODE.get()) {
            SmartDashboard.putNumber("InterpolationTesting/Shooter Applied Voltage",
                    shooterLeaderVoltage.getValueAsDouble());

            SmartDashboard.putNumber("Superstructure/Shooter/Leader Voltage (volts)",
                    shooterLeaderVoltage.getValueAsDouble());
            SmartDashboard.putNumber("Superstructure/Shooter/Leader Supply Current (amps)",
                    shooterLeadSupplyCurrent.getValueAsDouble());
            SmartDashboard.putNumber("Superstructure/Shooter/Leader Stator Current (amps)",
                    shooterLeadStatorCurrent.getValueAsDouble());

            SmartDashboard.putNumber("Superstructure/Shooter/Follower Voltage (volts)",
                    shooterFollowerVoltage.getValueAsDouble());
            SmartDashboard.putNumber("Superstructure/Shooter/Follower Supply Current (amps)",
                    shooterFollowSupplyCurrent.getValueAsDouble());
            SmartDashboard.putNumber("Superstructure/Shooter/Follower Stator Current (amps)",
                    shooterFollowStatorCurrent.getValueAsDouble());

            if (Robot.getMode() == RobotMode.DISABLED && !DriverStation.isFMSAttached()) {
                SmartDashboard.putBoolean(
                        "Robot/CAN/Main/Shooter Leader Motor Connected? (ID "
                                + String.valueOf(Ports.Superstructure.Shooter.MOTOR_LEAD) + ")",
                        shooterLeader.isConnected());
                SmartDashboard.putBoolean(
                        "Robot/CAN/Main/Shooter Follower Motor Connected? (ID "
                                + String.valueOf(Ports.Superstructure.Shooter.MOTOR_FOLLOW) + ")",
                        shooterFollower.isConnected());
            }
        }

        SmartDashboard.putNumber("InterpolationTesting/Shooter Closed Loop Error (RPM)",
                shooterLeaderClosedLoopError.getValueAsDouble() * 60.0);

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
        return Double.max(0, shooterLeadSupplyCurrent.getValueAsDouble()) +
                Double.max(0, shooterFollowSupplyCurrent.getValueAsDouble());
    }
}