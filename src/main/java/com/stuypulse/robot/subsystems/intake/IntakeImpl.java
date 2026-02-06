package com.stuypulse.robot.subsystems.intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeImpl extends Intake {
    private final TalonFX pivot;
    private final TalonFX rollerLeader;
    private final TalonFX rollerFollower;
    private final DutyCycleEncoder absoluteEncoder;

    public IntakeImpl() {
        super();

        pivot = new TalonFX(Ports.Intake.PIVOT);
        Motors.Intake.PIVOT_CONFIG.configure(pivot);

        rollerLeader = new TalonFX(Ports.Intake.ROLLER_LEADER);
        Motors.Intake.ROLLER_LEADER_CONFIG.configure(rollerLeader);

        rollerFollower = new TalonFX(Ports.Intake.ROLLER_FOLLOWER);
        Motors.Intake.ROLLER_FOLLOWER_CONFIG.configure(rollerFollower);

        absoluteEncoder = new DutyCycleEncoder(Ports.Intake.ABSOLUTE_ENCODER);
        absoluteEncoder.setInverted(false);

        rollerFollower.setControl(new Follower(Ports.Intake.ROLLER_LEADER, MotorAlignmentValue.Opposed));
    }

    @Override
    public boolean isAtTargetAngle() {
        return Math.abs(getCurrentAngle().getRadians() - getIntakeState().getTargetAngle().get().getRadians()) < Settings.Intake.PIVOT_ANGLE_TOLERANCE; 
    }

    @Override
    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(pivot.getPosition().getValueAsDouble());
    }

    
    public Rotation2d getAbsoluteAngle() {
        double angleRotations = absoluteEncoder.get() - Settings.Intake.PIVOT_ANGLE_OFFSET.getRotations();
        return Rotation2d.fromRotations(angleRotations > Settings.Intake.PIVOT_MAX_ANGLE.getRotations() + Settings.Intake.PIVOT_ANGLE_OFFSET.getRotations()
            ? angleRotations - 1
            : angleRotations);
    }

    @Override
    public void periodic() {
        // ROLLER CONTROLS
        pivot.setControl(new PositionVoltage(getIntakeState().getTargetAngle().get().getDegrees()));
        rollerLeader.setControl(new DutyCycleOut(getIntakeState().getTargetDutyCycle()));

        // PIVOT
        SmartDashboard.putString("Intake/Pivot/Current State", getIntakeState().toString());
        SmartDashboard.putBoolean("Intake/Pivot/At Target Angle", isAtTargetAngle());
        SmartDashboard.putNumber("Intake/Pivot/Current Velocity", pivot.getVelocity().getValueAsDouble());

        // ROLLERG
        SmartDashboard.putNumber("Intake/Roller/Duty Cycle Target Speed", getIntakeState().getTargetDutyCycle());
        SmartDashboard.putNumber("Intake/Roller/Current Velocity", rollerLeader.getVelocity().getValueAsDouble());
    }
}
