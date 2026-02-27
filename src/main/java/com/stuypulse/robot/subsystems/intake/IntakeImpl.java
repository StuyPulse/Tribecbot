/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.SettableNumber;
import com.stuypulse.robot.util.SysId;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import java.util.Optional;

import javax.xml.xpath.XPathExpressionException;

public class IntakeImpl extends Intake {
    private final TalonFX pivot;
    private final TalonFX rollerLeader;
    private final TalonFX rollerFollower;

    private final MotionMagicVoltage pivotController;
    private final DutyCycleOut rollerController;
    private final Follower follower;

    private SettableNumber velLimit;
    private SettableNumber accelLimit;

    private Optional<Double> pivotVoltageOverride;

    public IntakeImpl() {
        pivot = new TalonFX(Ports.Intake.PIVOT, Ports.RIO);
        Motors.Intake.PIVOT.configure(pivot);

        pivot.getConfigurator().apply(Motors.Intake.PIVOT_SLOT_0);

        rollerLeader = new TalonFX(Ports.Intake.ROLLER_LEADER, Ports.RIO);
        Motors.Intake.ROLLER.configure(rollerLeader);

        rollerFollower = new TalonFX(Ports.Intake.ROLLER_FOLLOWER, Ports.RIO);
        Motors.Intake.ROLLER.configure(rollerFollower);

        pivotController = new MotionMagicVoltage(getPivotState().getTargetAngle().getRotations())
            .withEnableFOC(true);
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
        Motors.Intake.PIVOT.withMotionProfile(velLimit.getRotations(), accelLimit.getRotations());
        Motors.Intake.PIVOT.configure(pivot);
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
            } else {
                // pivot.setControl(pivotController.withPosition(getPivotState().getTargetAngle().getRotations()));
                rollerLeader.setControl(rollerController.withOutput(getRollerState().getTargetDutyCycle()));
                rollerFollower.setControl(follower);
            }
        } else {
            pivot.stopMotor();
            rollerLeader.stopMotor();
            rollerFollower.stopMotor();
        }

        if (Settings.DEBUG_MODE) {
            // PIVOT
            SmartDashboard.putNumber("Intake/Pivot Voltage (volts)", pivot.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Intake/Pivot Current (amps)", pivot.getSupplyCurrent().getValueAsDouble());

            SmartDashboard.putNumber("Intake/Pivot Max Velocity Limit (deg/s)", velLimit.get());
            SmartDashboard.putNumber("Intake/Pivot Max Accel Limit (deg/s^2)", accelLimit.get());

            SmartDashboard.putNumber("Intake/Pivot Angle Error (deg)", Math.abs(getPivotState().getTargetAngle().getDegrees() - getPivotAngle().getDegrees()));

            // ROLLERS
            SmartDashboard.putNumber("Intake/Roller Leader Voltage (volts)", rollerLeader.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Intake/Roller Follower Voltage (volts)", rollerFollower.getMotorVoltage().getValueAsDouble());

            SmartDashboard.putNumber("Intake/Roller Leader Current (amps)", rollerLeader.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Intake/Roller Follower Current (amps)", rollerFollower.getSupplyCurrent().getValueAsDouble());
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
