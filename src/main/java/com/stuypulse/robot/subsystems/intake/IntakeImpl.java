/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.intake;

import java.util.Optional;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.RobotMode;
import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.PhoenixUtil;
import com.stuypulse.robot.util.SysId;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class IntakeImpl extends Intake {
    private Motors.TalonFXConfig pivotConfig;
    private Motors.TalonFXConfig rollerConfig;

    private final TalonFX pivot;
    private final TalonFX rollerLeader;
    private final TalonFX rollerFollower;

    private final DutyCycleOut rollerController;
    private final Follower follower;

    private Optional<Double> pivotVoltageOverride;
    private TorqueCurrentFOC torqueCurrentFOC;
    private VoltageOut voltageOut;
    private PositionVoltage positionVoltage;

    private BStream pivotStalling;

    private boolean isDigesting;

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

                .withPIDConstants(Gains.Intake.Pivot.slot0.kP.get(), Gains.Intake.Pivot.slot0.kI.get(), Gains.Intake.Pivot.slot0.kD.get(),
                        0)
                .withFFConstants(Gains.Intake.Pivot.slot0.kS.get(), Gains.Intake.Pivot.slot0.kV.get(), Gains.Intake.Pivot.slot0.kA.get(),
                        Gains.Intake.Pivot.slot0.kG, 0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign, 0)
                .withPIDConstants(Gains.Intake.Pivot.slot1.kP.get(), Gains.Intake.Pivot.slot1.kI.get(), Gains.Intake.Pivot.slot1.kD.get(),
                        1)
                .withFFConstants(Gains.Intake.Pivot.slot1.kS.get(), Gains.Intake.Pivot.slot1.kV.get(), Gains.Intake.Pivot.slot1.kA.get(),
                        Gains.Intake.Pivot.slot1.kG, 1)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign, 1)
                .withGravityType(GravityTypeValue.Arm_Cosine)

                .withSensorToMechanismRatio(Settings.Intake.GEAR_RATIO);

        rollerConfig = new Motors.TalonFXConfig()
                .withInvertedValue(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast)
                .withSupplyCurrentLimitAmps(37.0)
                .withStatorCurrentLimitEnabled(false)
                .withRampRate(0.50);

        torqueCurrentFOC = new TorqueCurrentFOC(0.0);
        voltageOut = new VoltageOut(0.0);
        positionVoltage = new PositionVoltage(0.0);

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

        isDigesting = false;

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
        PhoenixUtil.registerToRio(pivotSupplyCurrent, pivotStatorCurrent, pivotMotorPosition, rollerLeaderSupplyCurrent,
                rollerLeaderStatorCurrent, rollerFollowerSupplyCurrent, rollerFollowerStatorCurrent,
                rollerLeaderTemperature, rollerFollowerTemperature, pivotTemperature, pivotMotorVoltage,
                rollerLeaderVoltage, rollerFollowerVoltage);

        pivotStalling = BStream.create(
                () -> Math.abs(pivotSupplyCurrent.getValueAsDouble()) > Settings.Intake.PIVOT_STALL_CURRENT)
                .filtered(new BDebounce.Both(Settings.Intake.PIVOT_STALL_DEBOUNCE));
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
    public void setIntakeDigesting(boolean isDigesting) {
        this.isDigesting = isDigesting;
    }



    @Override
    public void periodicAfterScheduler() {
        super.periodicAfterScheduler();

        PivotState pivotState = getPivotState();
        RollerState rollerState = getRollerState();

        pivotConfig.updateGainsConfig(
                pivot,
                0,
                Gains.Intake.Pivot.slot0.kP,
                Gains.Intake.Pivot.slot0.kI,
                Gains.Intake.Pivot.slot0.kD,
                Gains.Intake.Pivot.slot0.kS,
                Gains.Intake.Pivot.slot0.kV,
                Gains.Intake.Pivot.slot0.kA);
        pivotConfig.updateGainsConfig(
                pivot,
                1,
                Gains.Intake.Pivot.slot1.kP,
                Gains.Intake.Pivot.slot1.kI,
                Gains.Intake.Pivot.slot1.kD,
                Gains.Intake.Pivot.slot1.kS,
                Gains.Intake.Pivot.slot1.kV,
                Gains.Intake.Pivot.slot1.kA);

        int slot = (isDigesting) ? 1 : 0;

        boolean applyingPushdownCurrent = false;
        if (EnabledSubsystems.INTAKE.get()) {
            if (pivotVoltageOverride.isPresent()) {
                pivot.setVoltage(pivotVoltageOverride.get());
            } else {
                // PIVOT
                if (pivotState == PivotState.DEPLOY && 
                    getPivotAngle().getDegrees() <= Settings.Intake.ANGLE_THRESHOLD_FOR_HOLDING_VOLTAGE.getDegrees()
                    && rollerState != RollerState.STOP && !isDigesting) {
                        // pivot.setControl(new VoltageOut(Settings.Intake.PUSHDOWN_VOLTAGE)); // applying 3 volts
                        double pushdownCurrent = 
                            Robot.getMode() == RobotMode.AUTON ? 
                            Settings.Intake.PUSHDOWN_CURRENT_AUTON : Settings.Intake.PUSHDOWN_CURRENT_TELEOP;
                        pivot.setControl(torqueCurrentFOC.withOutput(pushdownCurrent));
                        applyingPushdownCurrent = true;
                } else if (pivotState == PivotState.HOMING) {
                    pivot.setControl(voltageOut.withOutput(-Settings.Intake.HOMING_VOLTAGE));
                } else {
                    pivot.setControl(positionVoltage.withPosition(getPivotState().getTargetAngle().getRotations()).withSlot(slot));
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

        DogLog.log("Intake/Pivot Angle Error (deg)",
                Math.abs(pivotState.getTargetAngle().getDegrees() - getPivotAngle().getDegrees()));

        if (Robot.getPeriodicCounter() % Settings.LOGGING_FREQUENCY == 0) {

            // PIVOT
            DogLog.log("Intake/Pivot Pushdown Voltage Applied?", applyingPushdownCurrent);

            DogLog.log("Intake/Pivot Closed Loop Error (deg)",
                    pivot.getClosedLoopError().getValueAsDouble() * 360.0);

            if (Settings.DEBUG_MODE.get()) {
                DogLog.log("Intake/Voltage Override", pivotVoltageOverride.isPresent());
                DogLog.log("Intake/Pivot Temperature (C)", pivotTemperature.getValueAsDouble());
                DogLog.log("Intake/Leader Temperature (C)",
                        rollerLeaderTemperature.getValueAsDouble());
                DogLog.log("Intake/Follower Temperature (C)",
                        rollerFollowerTemperature.getValueAsDouble());

                // Rolers
                DogLog.log("Intake/Roller Leader Voltage (volts)",
                        rollerLeaderVoltage.getValueAsDouble());
                DogLog.log("Intake/Roller Leader Current (amps)",
                        rollerLeaderSupplyCurrent.getValueAsDouble());
                DogLog.log("Intake/Roller Leader Stator Current (amps)",
                        rollerLeaderStatorCurrent.getValueAsDouble());
                DogLog.log("Intake/Roller Follower Voltage (volts)",
                        rollerFollowerVoltage.getValueAsDouble());
                DogLog.log("Intake/Roller Follower Current (amps)",
                        rollerFollowerSupplyCurrent.getValueAsDouble());
                DogLog.log("Intake/Roller Follower Stator Current (amps)",
                        rollerFollowerStatorCurrent.getValueAsDouble());
                DogLog.log("Intake/Digesting", isDigesting);

                // Pivot
                DogLog.log("Intake/Pivot Voltage (volts)", pivotMotorVoltage.getValueAsDouble());
                DogLog.log("Intake/Pivot Supply Current (amps)",
                        pivotSupplyCurrent.getValueAsDouble());
                DogLog.log("Intake/Pivot Stator Current (amps)",
                        pivotStatorCurrent.getValueAsDouble());

                if (Robot.getMode() == RobotMode.DISABLED && !Robot.fmsAttached) {
                    DogLog.log("Robot/CAN/Main/Intake Pivot Motor Connected? (ID "
                            + String.valueOf(Ports.Intake.PIVOT) + ")", pivot.isConnected());
                    DogLog.log("Robot/CAN/Main/Intake Roller Leader Motor Connected? (ID "
                            + String.valueOf(Ports.Intake.ROLLER_LEADER) + ")", rollerLeader.isConnected());
                    DogLog.log("Robot/CAN/Main/Intake Roller Follower Motor Connected? (ID "
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
