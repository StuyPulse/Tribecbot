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

public class IntakeImpl extends Intake {
    private final TalonFX pivot;
    private final TalonFX rollerLeader;
    private final TalonFX rollerFollower;

    private final MotionMagicVoltage pivotController;
    private final DutyCycleOut rollerController;
    private final Follower follower;

    private Optional<Double> pivotVoltageOverride;

    public IntakeImpl() {
        pivot = new TalonFX(Ports.Intake.PIVOT);
        Motors.Intake.PIVOT.configure(pivot);

        rollerLeader = new TalonFX(Ports.Intake.ROLLER_LEADER);
        Motors.Intake.ROLLER.configure(rollerLeader);

        rollerFollower = new TalonFX(Ports.Intake.ROLLER_FOLLOWER);
        Motors.Intake.ROLLER.configure(rollerFollower);

        pivotController = new MotionMagicVoltage(getPivotState().getTargetAngle().getRotations())
            .withEnableFOC(true);
        rollerController = new DutyCycleOut(getRollerState().getTargetDutyCycle())
            .withEnableFOC(true);
        follower = new Follower(Ports.Intake.ROLLER_LEADER, MotorAlignmentValue.Opposed);

        pivotVoltageOverride = Optional.empty();
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
    
    @Override
    public void periodic() {
        super.periodic();

        if (EnabledSubsystems.INTAKE.get()) {
            if (pivotVoltageOverride.isPresent()) {
                pivot.setVoltage(pivotVoltageOverride.get());
            } else {
                pivot.setControl(pivotController.withPosition(getPivotState().getTargetAngle().getRotations()));
                rollerLeader.setControl(rollerController.withOutput(getRollerState().getTargetDutyCycle()));
                rollerFollower.setControl(follower);
            }
        } else {
            pivot.stopMotor();
            rollerLeader.stopMotor();
            rollerFollower.stopMotor();
        }

        SmartDashboard.putNumber("Intake/Pivot Angle Error (deg)",
            Math.abs(getPivotState().getTargetAngle().getDegrees() - getPivotAngle().getDegrees()));

        if (Settings.DEBUG_MODE) {
            // PIVOT
            SmartDashboard.putNumber("Intake/Pivot Voltage (volts)", pivot.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Intake/Pivot Current (amps)", pivot.getSupplyCurrent().getValueAsDouble());

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
