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
        pivot = new TalonFX(Ports.Intake.PIVOT);
        Motors.Intake.PIVOT_CONFIG.configure(pivot);

        rollerLeader = new TalonFX(Ports.Intake.ROLLER_LEADER);
        Motors.Intake.ROLLER_LEADER_CONFIG.configure(rollerLeader);

        rollerFollower = new TalonFX(Ports.Intake.ROLLER_FOLLOWER);
        Motors.Intake.ROLLER_FOLLOWER_CONFIG.configure(rollerFollower);

        absoluteEncoder = new DutyCycleEncoder(Ports.Intake.ABSOLUTE_ENCODER, 1.00, Settings.Intake.PIVOT_ANGLE_OFFSET.getRotations()); // TODO: Set Full Range, Verify Expected_Zero wants a Rotation and not Degrees
    }

    /**
     * Returns whether the current Intake angle is within a tolerance of the target angle
     * @return Bool: At Target Angles
     */
    @Override
    public boolean isAtTargetAngle() {
        return Math.abs((getAbsoluteAngle().getRotations()) - getIntakeState().getTargetAngle().getRotations()) < Settings.Intake.PIVOT_ANGLE_TOLERANCE.getRotations(); 
    }

    /**
     * Gets the Current Intake Pivot Angle using the Relative Encoder built into the Kraken
     * @return Rotation2d: Current Angle (Relative)
     */
    @Override
    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(pivot.getPosition().getValueAsDouble());
    }

    /**
     * Gets the Current Intake Pivot Angle using the Absolute Encoder on the Intake
     * @return Rotation2d: Current Angle (Absolute)
     */
    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromRotations(absoluteEncoder.get());
    }

    /**
     * Runs Motors based off Target Values of the Target State
     */
    private void runMotors() {
        pivot.setControl(new PositionVoltage(getIntakeState().getTargetAngle().getDegrees())); // TODO: make this use the absolute encoder to ensure it works even when belt skip
        rollerLeader.setControl(new DutyCycleOut(getIntakeState().getTargetDutyCycle()));
        rollerFollower.setControl(new Follower(Ports.Intake.ROLLER_LEADER, MotorAlignmentValue.Opposed));
    }

    @Override
    public void periodic() {
        // ROLLER CONTROLS
        if (Settings.EnabledSubsystems.INTAKE.getAsBoolean()) {
            runMotors();
        } else {
            pivot.setControl(new DutyCycleOut(0));
            rollerLeader.setControl(new DutyCycleOut(0));
            rollerFollower.setControl(new DutyCycleOut(0));
        }

        // DEBUG
        if (Settings.DEBUG_MODE) { // TODO: Make some of these always shown (Not in debug mode)

            // PIVOT
            SmartDashboard.putString("Intake/Pivot/Current State", getIntakeState().toString());
            SmartDashboard.putBoolean("Intake/Pivot/At Target Angle", isAtTargetAngle());
            SmartDashboard.putNumber("Intake/Pivot/Current Velocity", pivot.getVelocity().getValueAsDouble());
            SmartDashboard.putNumber("Intake/Pivot/Current Angle (Relative Encoder)", getCurrentAngle().getDegrees());
            SmartDashboard.putNumber("Intake/Pivot/Current Angle (Absolute Encoder)", getAbsoluteAngle().getDegrees());

            // ROLLERS
            SmartDashboard.putNumber("Intake/Roller/Duty Cycle Target Speed", getIntakeState().getTargetDutyCycle());
            SmartDashboard.putNumber("Intake/Roller/Current Velocity", rollerLeader.getVelocity().getValueAsDouble());

        }
    }
}
