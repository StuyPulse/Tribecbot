/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.RobotMode;
import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.PhoenixUtil;
import com.stuypulse.robot.util.SysId;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import java.util.Optional;

public class IntakeImpl extends Intake {
    private Motors.TalonFXConfig pivotConfig;
    private Motors.TalonFXConfig rollerConfig;

    private final TalonFX pivot;
    private final TalonFX rollerLeader;
    private final TalonFX rollerFollower;

    private final DutyCycleOut rollerController;
    private final Follower follower;

    private Optional<Double> pivotVoltageOverride;

    private BStream pivotStalling;

    StatusSignal<Current> pivotSupplyCurrent;
    StatusSignal<Current> pivotStatorCurrent;
    StatusSignal<Current> rollerLeaderSupplyCurrent;
    StatusSignal<Current> rollerLeaderStatorCurrent;
    StatusSignal<Current> rollerFollowerSupplyCurrent;
    StatusSignal<Current> rollerFollowerStatorCurrent;
    StatusSignal<Temperature> rollerLeaderTemperature;
    StatusSignal<Temperature> rollerFollowerTemperature;
    StatusSignal<Temperature> pivotTemperature;
    StatusSignal<Angle> pivotMotorPosition;
    StatusSignal<Voltage> pivotMotorVoltage;
    StatusSignal<Voltage> rollerLeaderVoltage;
    StatusSignal<Voltage> rollerFollowerVoltage;

    public IntakeImpl() {
        pivotConfig = new Motors.TalonFXConfig()
                .withInvertedValue(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)

                .withSupplyCurrentLimitAmps(10.0) // was 60 on practice day
                .withStatorCurrentLimitEnabled(false)
                .withRampRate(0.25)

                .withPIDConstants(Gains.Intake.Pivot.kP.get(), Gains.Intake.Pivot.kI.get(), Gains.Intake.Pivot.kD.get(),
                        0)
                .withFFConstants(Gains.Intake.Pivot.kS.get(), Gains.Intake.Pivot.kV.get(), Gains.Intake.Pivot.kA.get(),
                        Gains.Intake.Pivot.kG, 0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign, 0)
                .withGravityType(GravityTypeValue.Arm_Cosine)

                .withSensorToMechanismRatio(Settings.Intake.GEAR_RATIO);

        rollerConfig = new Motors.TalonFXConfig()
                .withInvertedValue(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)
                .withSupplyCurrentLimitAmps(30.0)
                .withStatorCurrentLimitEnabled(false)
                .withRampRate(0.50);

        pivot = new TalonFX(Ports.Intake.PIVOT, Ports.RIO);
        pivotConfig.configure(pivot);

        rollerLeader = new TalonFX(Ports.Intake.ROLLER_LEADER, Ports.RIO);
        rollerConfig.configure(rollerLeader);

        rollerFollower = new TalonFX(Ports.Intake.ROLLER_FOLLOWER, Ports.RIO);
        rollerConfig.configure(rollerFollower);

        rollerController = new DutyCycleOut(getRollerState().getTargetDutyCycle()).withEnableFOC(true);
        follower = new Follower(Ports.Intake.ROLLER_LEADER, MotorAlignmentValue.Opposed);

        rollerFollower.setControl(follower);

        pivotVoltageOverride = Optional.empty();

        pivot.setPosition(Settings.Intake.PIVOT_MAX_ANGLE.getRotations());

        pivotSupplyCurrent = pivot.getSupplyCurrent();
        pivotStatorCurrent = pivot.getStatorCurrent();
        pivotMotorPosition = pivot.getPosition();
        rollerLeaderSupplyCurrent = rollerLeader.getSupplyCurrent();
        rollerLeaderStatorCurrent = rollerLeader.getStatorCurrent();
        rollerFollowerSupplyCurrent = rollerFollower.getSupplyCurrent();
        rollerFollowerStatorCurrent = rollerFollower.getStatorCurrent();
        rollerLeaderTemperature = rollerLeader.getDeviceTemp();
        rollerFollowerTemperature = rollerFollower.getDeviceTemp();
        pivotTemperature = pivot.getDeviceTemp();
        pivotMotorVoltage = pivot.getMotorVoltage();
        rollerLeaderVoltage = rollerLeader.getMotorVoltage();
        rollerFollowerVoltage = rollerFollower.getMotorVoltage();
        PhoenixUtil.registerToRio(pivotStatorCurrent, rollerLeaderSupplyCurrent,
                rollerLeaderStatorCurrent, rollerFollowerSupplyCurrent, rollerFollowerStatorCurrent,
                rollerLeaderTemperature, rollerFollowerTemperature, pivotTemperature, pivotMotorVoltage,
                rollerLeaderVoltage, rollerFollowerVoltage, pivotMotorPosition );

        pivotStalling = BStream.create(
                () -> Math.abs(pivotSupplyCurrent.getValueAsDouble()) > Settings.Intake.STALL_CURRENT_LIMIT)
                .filtered(new BDebounce.Both(Settings.Intake.STALL_DEBOUNCE));
    }

    @Override
    public boolean pivotStalling() {
        return pivotStalling.get();
    }

    @Override
    public boolean pivotAtTolerance() {
        double error = getPivotAngle().minus(getPivotState().getTargetAngle()).getRotations();
        return Math.abs(error) < Settings.Intake.PIVOT_ANGLE_TOLERANCE.getRotations();
    }

    @Override
    public Rotation2d getPivotAngle() {
        return Rotation2d.fromRotations(pivotMotorPosition.getValueAsDouble());
    }

    @Override
    public void seedPivotStowed() {
        pivot.setPosition(Settings.Intake.PIVOT_MAX_ANGLE.getRotations());
    }

    @Override
    public void seedPivotDeployed() {
        pivot.setPosition(Settings.Intake.PIVOT_MIN_ANGLE.getRotations());
    }

    @Override
    public void periodicAfterScheduler() {
        super.periodicAfterScheduler();

        PivotState pivotState = getPivotState();
        RollerState rollerState = getRollerState();

        pivotConfig.updateGainsConfig(
                pivot,
                0,
                Gains.Intake.Pivot.kP,
                Gains.Intake.Pivot.kI,
                Gains.Intake.Pivot.kD,
                Gains.Intake.Pivot.kS,
                Gains.Intake.Pivot.kV,
                Gains.Intake.Pivot.kA);

        boolean applyingPushdownVoltage = false;
        if (EnabledSubsystems.INTAKE.get()) {
            if (pivotVoltageOverride.isPresent()) {
                pivot.setVoltage(pivotVoltageOverride.get());
            } else {
                // PIVOT
                if (pivotState == PivotState.DEPLOY && 
                    getPivotAngle().getDegrees() <= Settings.Intake.ANGLE_THRESHOLD_FOR_HOLDING_VOLTAGE.getDegrees()
                    && rollerState != RollerState.STOP) {
                        pivot.setControl(new VoltageOut(-Settings.Intake.PUSHDOWN_VOLTAGE)); // applying 3 volts
                        applyingPushdownVoltage = true;
                } else if (pivotState == PivotState.HOMING) {
                    pivot.setControl(new VoltageOut(-Settings.Intake.HOMING_VOLTAGE));
                } else {
                    pivot.setControl(new PositionVoltage(pivotState.getTargetAngle().getRotations()));
                }

                // ROLLERS
                if (pivotState == PivotState.DEPLOY
                        && getPivotAngle().getDegrees() <= Settings.Intake.THRESHOLD_TO_START_ROLLERS.getDegrees()) {
                    rollerLeader.setControl(rollerController.withOutput(getRollerState().getTargetDutyCycle()));
                } else {
                    rollerLeader.stopMotor();
                }
                rollerFollower.setControl(follower);
            }

            if (pivotState == PivotState.HOMING && pivotStalling()) {
                seedPivotDeployed();
            }
        } else {
            pivot.stopMotor();
            rollerLeader.stopMotor();
            rollerFollower.stopMotor();
        }

        SmartDashboard.putNumber("Intake/Pivot Angle Error (deg)",
                Math.abs(pivotState.getTargetAngle().getDegrees() - getPivotAngle().getDegrees()));

        if (Robot.getPeriodicCounter() % Settings.LOGGING_FREQUENCY == 0) {

            // PIVOT
            SmartDashboard.putBoolean("Intake/Pivot Pushdown Voltage Applied?", applyingPushdownVoltage);

            SmartDashboard.putNumber("Intake/Pivot Closed Loop Error (deg)",
                    pivot.getClosedLoopError().getValueAsDouble() * 360.0);

            if (Settings.DEBUG_MODE.get()) {
                SmartDashboard.putBoolean("Intake/Voltage Override", pivotVoltageOverride.isPresent());
                SmartDashboard.putNumber("Intake/Pivot Temperature (C)", pivotTemperature.getValueAsDouble());
                SmartDashboard.putNumber("Intake/Leader Temperature (C)",
                        rollerLeaderTemperature.getValueAsDouble());
                SmartDashboard.putNumber("Intake/Follower Temperature (C)",
                        rollerFollowerTemperature.getValueAsDouble());

                // Rolers
                SmartDashboard.putNumber("Intake/Roller Leader Voltage (volts)",
                        rollerLeaderVoltage.getValueAsDouble());
                SmartDashboard.putNumber("Intake/Roller Leader Current (amps)",
                        rollerLeaderSupplyCurrent.getValueAsDouble());
                SmartDashboard.putNumber("Intake/Roller Leader Stator Current (amps)",
                        rollerLeaderStatorCurrent.getValueAsDouble());
                SmartDashboard.putNumber("Intake/Roller Follower Voltage (volts)",
                        rollerFollowerVoltage.getValueAsDouble());
                SmartDashboard.putNumber("Intake/Roller Follower Current (amps)",
                        rollerFollowerSupplyCurrent.getValueAsDouble());
                SmartDashboard.putNumber("Intake/Roller Follower Stator Current (amps)",
                        rollerFollowerStatorCurrent.getValueAsDouble());

                // Pivot
                SmartDashboard.putNumber("Intake/Pivot Voltage (volts)", pivotMotorVoltage.getValueAsDouble());
                SmartDashboard.putNumber("Intake/Pivot Supply Current (amps)",
                        pivotSupplyCurrent.getValueAsDouble());
                SmartDashboard.putNumber("Intake/Pivot Stator Current (amps)",
                        pivotStatorCurrent.getValueAsDouble());

                if (Robot.getMode() == RobotMode.DISABLED && !DriverStation.isFMSAttached()) {
                    SmartDashboard.putBoolean("Robot/CAN/Main/Intake Pivot Motor Connected? (ID "
                            + String.valueOf(Ports.Intake.PIVOT) + ")", pivot.isConnected());
                    SmartDashboard.putBoolean("Robot/CAN/Main/Intake Roller Leader Motor Connected? (ID "
                            + String.valueOf(Ports.Intake.ROLLER_LEADER) + ")", rollerLeader.isConnected());
                    SmartDashboard.putBoolean("Robot/CAN/Main/Intake Roller Follower Motor Connected? (ID "
                            + String.valueOf(Ports.Intake.ROLLER_FOLLOWER) + ")", rollerFollower.isConnected());
                }
                Robot.getEnergyUtil().logEnergyUsage(getName(), getCurrentDraw());

            }
        }
    }

    @Override
    public void setPivotVoltageOverride(Optional<Double> voltage) {
        this.pivotVoltageOverride = voltage;
    }

    @Override
    public SysIdRoutine getPivotSysIdRoutine() {
        return SysId.getRoutine(
                2,
                6,
                "Intake Pivot",
                voltage -> setPivotVoltageOverride(Optional.of(voltage)),
                () -> pivot.getPosition().getValueAsDouble(),
                () -> pivot.getVelocity().getValueAsDouble(),
                () -> pivot.getMotorVoltage().getValueAsDouble(),
                getInstance());
    }

    @Override
    public double getCurrentDraw() {
        return Double.max(0, pivotSupplyCurrent.getValueAsDouble()) +
                Double.max(0, rollerFollowerSupplyCurrent.getValueAsDouble()) +
                Double.max(0, rollerLeaderSupplyCurrent.getValueAsDouble());
    }
}
