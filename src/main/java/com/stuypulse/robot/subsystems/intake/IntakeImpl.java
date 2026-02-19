package com.stuypulse.robot.subsystems.intake;

import java.util.Optional;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.SysId;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class IntakeImpl extends Intake {
    private final TalonFX pivot;
    private final TalonFX rollerLeader;
    private final TalonFX rollerFollower;

    private final DutyCycleOut rollerController;
    private final MotionMagicVoltage pivotController;

    private final Follower follower;

    private Optional<Double> pivotVoltageOverride;

    public IntakeImpl() {
        pivot = new TalonFX(Ports.Intake.PIVOT);
        Motors.Intake.PIVOT_CONFIG.configure(pivot);

        rollerLeader = new TalonFX(Ports.Intake.ROLLER_LEADER);
        Motors.Intake.ROLLER_LEADER_CONFIG.configure(rollerLeader);

        rollerFollower = new TalonFX(Ports.Intake.ROLLER_FOLLOWER);
        Motors.Intake.ROLLER_FOLLOWER_CONFIG.configure(rollerFollower);

        rollerController = new DutyCycleOut(0.0);
        pivotController = new MotionMagicVoltage(getState().getTargetAngle().getRotations());
        follower = new Follower(Ports.Intake.ROLLER_LEADER, MotorAlignmentValue.Opposed);

        pivotVoltageOverride = Optional.empty();
    }

    @Override
    public boolean pivotAtTolerance() {
        return Math.abs(
            (getPivotAngle().getRotations()) - getState().getTargetAngle().getRotations()) 
                < Settings.Intake.PIVOT_ANGLE_TOLERANCE.getRotations();
    }

    public Rotation2d getPivotAngle() {
        return Rotation2d.fromRotations(pivot.getPosition().getValueAsDouble());
    }

    @Override
    public void periodic() {
        if (Settings.EnabledSubsystems.INTAKE.get()) {
            if (pivotVoltageOverride.isPresent()) {
                pivot.setVoltage(pivotVoltageOverride.get());
            } else {
                pivot.setControl(pivotController.withPosition(getState().getTargetAngle().getRotations()));
                rollerLeader.setControl(rollerController.withOutput(getState().getTargetDutyCycle()));
                rollerFollower.setControl(follower);
            }
        } else {
            pivot.stopMotor();
            rollerLeader.stopMotor();
            rollerFollower.stopMotor();
        }

        SmartDashboard.putNumber("Intake/Pivot Angle Error (deg)",
                Math.abs(getState().getTargetAngle().getDegrees() - getPivotAngle().getDegrees()));

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
