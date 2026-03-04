/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.intake;

import java.util.Optional;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.SettableNumber;
import com.stuypulse.robot.util.SysId;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class IntakeImpl extends Intake {
    private final Motors.TalonFXConfig pivotConfig;
    private final Motors.TalonFXConfig rollerConfig;

    private final TalonFX pivot;
    private final TalonFX rollerLeader;
    private final TalonFX rollerFollower;

    private final MotionMagicVoltage pivotController;

    private final DutyCycleOut rollerController;
    private final Follower follower;

    private SettableNumber velLimit;
    private SettableNumber accelLimit;

    private Optional<Double> pivotVoltageOverride;

    private BStream pivotStalling;

    public IntakeImpl() {
        pivotConfig = new Motors.TalonFXConfig()
            .withInvertedValue(InvertedValue.Clockwise_Positive)
            
            .withSupplyCurrentLimitAmps(60)
            .withStatorCurrentLimitEnabled(false)
            
            .withPIDConstants(Gains.Intake.Pivot.kP, Gains.Intake.Pivot.kI, Gains.Intake.Pivot.kD, 0)
            .withFFConstants(Gains.Intake.Pivot.kS, Gains.Intake.Pivot.kV, Gains.Intake.Pivot.kA, Gains.Intake.Pivot.kG, 0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign, 0)
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withMotionProfile(Settings.Intake.PIVOT_MAX_VEL_STOW.getRotations(), Settings.Intake.PIVOT_MAX_ACCEL_STOW.getRotations())
            
            .withNeutralMode(NeutralModeValue.Brake)
            .withRampRate(0.25)
            
            .withSensorToMechanismRatio(Settings.Intake.GEAR_RATIO);

        rollerConfig = new Motors.TalonFXConfig()
            .withInvertedValue(InvertedValue.CounterClockwise_Positive)

            .withSupplyCurrentLimitAmps(45)
            .withStatorCurrentLimitEnabled(false)
            
            .withRampRate(0.50)
            .withNeutralMode(NeutralModeValue.Coast);

        pivot = new TalonFX(Ports.Intake.PIVOT, Ports.RIO);
        pivotConfig.configure(pivot);

        rollerLeader = new TalonFX(Ports.Intake.ROLLER_LEADER, Ports.RIO);
        rollerConfig.configure(rollerLeader);

        rollerFollower = new TalonFX(Ports.Intake.ROLLER_FOLLOWER, Ports.RIO);
        rollerConfig.configure(rollerFollower);

        pivotController = new MotionMagicVoltage(getPivotState().getTargetAngle().getRotations())
            .withEnableFOC(true);

        rollerController = new DutyCycleOut(getRollerState().getTargetDutyCycle())
            .withEnableFOC(true);
        follower = new Follower(Ports.Intake.ROLLER_LEADER, MotorAlignmentValue.Aligned);

        rollerFollower.setControl(follower);

        velLimit = new SettableNumber(Settings.Intake.PIVOT_MAX_VEL_DEPLOY.getDegrees());
        accelLimit = new SettableNumber(Settings.Intake.PIVOT_MAX_ACCEL_DEPLOY.getDegrees());

        pivotVoltageOverride = Optional.empty();

        pivot.setPosition(Settings.Intake.PIVOT_MAX_ANGLE.getRotations());

        pivotStalling = BStream.create(
            () -> Math.abs(pivot.getSupplyCurrent().getValueAsDouble()) > Settings.Intake.STALL_CURRENT_LIMIT)
            .filtered(new BDebounce.Both(1.0));

    }

    @Override 
    public boolean pivotStalling() { 
        // return Math.abs(pivot.getSupplyCurrent().getValueAsDouble()) > Settings.Intake.debugCurrentLimit;
        return pivotStalling.get();
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
    public void zeroPivotStowed() {
        pivot.setPosition(Settings.Intake.PIVOT_MAX_ANGLE.getRotations());
    }

    @Override
    public void zeroPivotDeployed() {
        pivot.setPosition(Settings.Intake.PIVOT_MIN_ANGLE.getRotations());
    } 

    @Override
    public void periodic() {
        super.periodic();

        if (EnabledSubsystems.INTAKE.get()) {

            if (getPivotState() == PivotState.ANALOG) { // comment out the setControl line if it breaks
                pivot.setControl(pivotController.withPosition(driverInputToAngle().getRotations()));
            }
            else if (getPivotState() == PivotState.DEBUG) { 
                pivot.setControl(new VoltageOut(Settings.Intake.debugVoltage)); //TODO: check if we want DEBUG enum/state
                if (pivotStalling.get()) { //TODO: update what we want the intake to do if 
                    pivot.setPosition(0);
                }
            }
            
            else if (pivotVoltageOverride.isPresent()) {
                pivot.setVoltage(pivotVoltageOverride.get());
            }
            
            else {
                if (getPivotState() == PivotState.DEPLOY && getPivotAngle().getDegrees() <= Settings.Intake.ARBITRARY_VOLTAGE_THRESHOLD.getDegrees()) {
                    pivot.setControl(new VoltageOut(-Settings.Intake.PUSHDOWN_VOLTAGE)); // applying 3 volts 
                }

                else {
                    pivot.setControl(new PositionVoltage(getPivotState().getTargetAngle().getRotations()));
                }

                if (getPivotAngle().getDegrees() <= Settings.Intake.THRESHHOLD_TO_START_ROLLERS.getDegrees()) {
                    rollerLeader.setControl(rollerController.withOutput(getRollerState().getTargetDutyCycle()));
                    rollerFollower.setControl(follower);
                }

                else {
                    rollerLeader.stopMotor();
                    rollerFollower.stopMotor();
                }
            }
        } else {
            pivot.stopMotor();
            rollerLeader.stopMotor();
            rollerFollower.stopMotor();
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

            SmartDashboard.putNumber("Intake/Pivot Closed Loop Error (deg)", pivot.getClosedLoopError().getValueAsDouble() * 360.0);
            

            // ROLLERS
            SmartDashboard.putNumber("Intake/Roller Leader Voltage (volts)", rollerLeader.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Intake/Roller Follower Voltage (volts)", rollerFollower.getMotorVoltage().getValueAsDouble());

            SmartDashboard.putNumber("Intake/Roller Leader Current (amps)", rollerLeader.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Intake/Roller Follower Current (amps)", rollerFollower.getSupplyCurrent().getValueAsDouble());

            SmartDashboard.putNumber("Intake/Pivot Temperature (C)", pivot.getDeviceTemp().getValueAsDouble());
            SmartDashboard.putNumber("Intake/Leader Temperature (C)", rollerLeader.getDeviceTemp().getValueAsDouble());
            SmartDashboard.putNumber("Intake/Follower Temperature (C)", rollerFollower.getDeviceTemp().getValueAsDouble());

            SmartDashboard.putNumber("Current Draws/Intake Pivot (amps)", pivot.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Current Draws/Intake Roller Leader (amps)", rollerLeader.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Current Draws/Intake Roller Follower (amps)", rollerFollower.getSupplyCurrent().getValueAsDouble());
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
