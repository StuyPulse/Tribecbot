/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.SettableNumber;
import com.stuypulse.robot.util.SysId;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import java.util.Optional;

public class IntakeImpl extends Intake {
    private final Motors.TalonFXConfig pivotConfig;
    private final Motors.TalonFXConfig rollerConfig;

    private final TalonFX pivot;
    private final TalonFX rollerLeader;
    private final TalonFX rollerFollower;

    private final MotionMagicVoltage pivotController;
    private final BangBangController pivotAggressiveController;

    private final DutyCycleOut rollerController;
    private final Follower follower;

    private SettableNumber velLimit;
    private SettableNumber accelLimit;

    private Optional<Double> pivotVoltageOverride;

    public IntakeImpl() {
        pivotConfig = new Motors.TalonFXConfig()
            .withCurrentLimitAmps(60.0)
            .withRampRate(0.25)
            .withNeutralMode(NeutralModeValue.Brake)
            .withInvertedValue(InvertedValue.Clockwise_Positive)
            .withPIDConstants(Gains.Intake.Pivot.kP, Gains.Intake.Pivot.kI, Gains.Intake.Pivot.kD, 0)
            .withFFConstants(Gains.Intake.Pivot.kS, Gains.Intake.Pivot.kV, Gains.Intake.Pivot.kA, Gains.Intake.Pivot.kG, 0)
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign, 0)
            .withSensorToMechanismRatio(Settings.Intake.GEAR_RATIO)
            .withMotionProfile(Settings.Intake.PIVOT_MAX_VEL_STOW.getRotations(), Settings.Intake.PIVOT_MAX_ACCEL_STOW.getRotations());

        rollerConfig = new Motors.TalonFXConfig()
            .withCurrentLimitAmps(60.0)
            .withRampRate(0.50)
            .withNeutralMode(NeutralModeValue.Coast)
            .withInvertedValue(InvertedValue.CounterClockwise_Positive);

        pivot = new TalonFX(Ports.Intake.PIVOT, Ports.RIO);
        pivotConfig.configure(pivot);

        rollerLeader = new TalonFX(Ports.Intake.ROLLER_LEADER, Ports.RIO);
        rollerConfig.configure(rollerLeader);

        rollerFollower = new TalonFX(Ports.Intake.ROLLER_FOLLOWER, Ports.RIO);
        rollerConfig.configure(rollerFollower);

        pivotController = new MotionMagicVoltage(getPivotState().getTargetAngle().getRotations())
            .withEnableFOC(true);

        pivotAggressiveController = new BangBangController(Settings.Intake.PIVOT_ANGLE_TOLERANCE.getRotations());

        rollerController = new DutyCycleOut(getRollerState().getTargetDutyCycle())
            .withEnableFOC(true);
        follower = new Follower(Ports.Intake.ROLLER_LEADER, MotorAlignmentValue.Aligned);

        velLimit = new SettableNumber(Settings.Intake.PIVOT_MAX_VEL_DEPLOY.getDegrees());
        accelLimit = new SettableNumber(Settings.Intake.PIVOT_MAX_ACCEL_DEPLOY.getDegrees());

        pivotVoltageOverride = Optional.empty();

        pivot.setPosition(Settings.Intake.PIVOT_MAX_ANGLE.getRotations());
    }

    @Override
    public boolean pivotAtTolerance() {
        return Math.abs(
            getPivotAngle().getRotations() - getPivotState().getTargetAngle().getRotations())
            < Settings.Intake.PIVOT_ANGLE_TOLERANCE.getRotations();
    }

    @Override
    public Rotation2d getPivotAngle() {
        return Rotation2d.fromRotations(pivot.getPosition().getValueAsDouble());
    }

    public void setMotionProfileConstraints(Rotation2d velLimit, Rotation2d accelLimit) {
        this.velLimit.set(velLimit.getDegrees());
        this.accelLimit.set(accelLimit.getDegrees());
        pivotConfig.withMotionProfile(velLimit.getRotations(), accelLimit.getRotations());
        pivotConfig.configure(pivot);
    }
    
    @Override
    public void setPivotState(PivotState pivotState) {
        super.setPivotState(pivotState);
        if (getPivotState() == PivotState.STOW) {
            SmartDashboard.putString("Intake/Profile Constraints", "S");
            setMotionProfileConstraints(Settings.Intake.PIVOT_MAX_VEL_STOW, Settings.Intake.PIVOT_MAX_ACCEL_STOW);
        } else if (getPivotState() == PivotState.DEPLOY) {
            SmartDashboard.putString("Intake/Profile Constraints", "D");
            setMotionProfileConstraints(Settings.Intake.PIVOT_MAX_VEL_DEPLOY, Settings.Intake.PIVOT_MAX_ACCEL_DEPLOY);
        }
    }

    @Override
    public void periodic() {
        super.periodic();

        if (EnabledSubsystems.INTAKE.get()) {
            if (getPivotState() == PivotState.ANALOG) { // comment out the setControl line if it breaks
                pivot.setControl(pivotController.withPosition(driverInputToAngle().getRotations()));
            }
            
            else if (pivotVoltageOverride.isPresent()) {
                pivot.setVoltage(pivotVoltageOverride.get());
            }
            
            // else if (getPivotState() == PivotState.BANGBANG) {
            //     pivotAggressiveController.calculate(getPivotAngle().getRotations(), getPivotState().getTargetAngle().getRotations());
            //     pivot.setControl(new VelocityVoltage(pivotAggressiveController.getMeasurement() * 12));

            // TODO: might have to calculate inside of pivot.setcontrol
            //     //TODO: finish and apply to motor -> motor config conflict
            // }
            else {
                pivot.setControl(new PositionVoltage(getPivotState().getTargetAngle().getRotations()));
                //pivot.setControl(pivotController.withPosition(getPivotState().getTargetAngle().getRotations()));
                rollerLeader.setControl(rollerController.withOutput(getRollerState().getTargetDutyCycle()));
                rollerFollower.setControl(follower);
            }
        } else {
            // pivot.stopMotor();
            // rollerLeader.stopMotor();
            // rollerFollower.stopMotor();
        }

        if (Settings.DEBUG_MODE) {
            // PIVOT
            SmartDashboard.putBoolean("Intake/Voltage Override", pivotVoltageOverride.isPresent());

            SmartDashboard.putNumber("Intake/Debug", getPivotState().getTargetAngle().getRotations());

            SmartDashboard.putNumber("Intake/Pivot Voltage (volts)", pivot.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Intake/Pivot Supply Current (amps)", pivot.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Intake/Pivot Stator Current (amps)", pivot.getStatorCurrent().getValueAsDouble());

            SmartDashboard.putNumber("Intake/Pivot Max Velocity Limit (deg/s)", velLimit.get());
            SmartDashboard.putNumber("Intake/Pivot Max Accel Limit (deg/s^2)", accelLimit.get());

            SmartDashboard.putNumber("Intake/Pivot Angle Error (deg)", Math.abs(getPivotState().getTargetAngle().getDegrees() - getPivotAngle().getDegrees()));

            //BANGBANG
            SmartDashboard.putNumber("Intake/Pivot BANGANG tolerance", pivotAggressiveController.getSetpoint());
            SmartDashboard.putNumber("Intake/Pivot BANGANG measurement (0 - 1)", pivotAggressiveController.getMeasurement());
            SmartDashboard.putNumber("Intake/Pivot BANGANG Distance between target and setpoint", pivotAggressiveController.getError());
            //TODO: add whatever was still not added 
            

            // ROLLERS
            SmartDashboard.putNumber("Intake/Roller Leader Voltage (volts)", rollerLeader.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Intake/Roller Follower Voltage (volts)", rollerFollower.getMotorVoltage().getValueAsDouble());

            SmartDashboard.putNumber("Intake/Roller Leader Supply Current (amps)", rollerLeader.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Intake/Roller Leader Stator Current (amps)", rollerLeader.getStatorCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Intake/Roller Follower Supply Current (amps)", rollerFollower.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Intake/Roller Follower Stator Current (amps)", rollerFollower.getStatorCurrent().getValueAsDouble());
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
            () -> getPivotAngle().getRotations(),
            () -> pivot.getVelocity().getValueAsDouble(),
            () -> pivot.getMotorVoltage().getValueAsDouble(),
            getInstance());
    }
}
