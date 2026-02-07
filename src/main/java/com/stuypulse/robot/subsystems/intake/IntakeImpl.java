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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeImpl extends Intake {
    private final TalonFX pivot;
    private final TalonFX rollerLeader;
    private final TalonFX rollerFollower;
    private final DutyCycleEncoder absoluteEncoder;

    private final DutyCycleOut rollerDutyCycleController;

    private final DutyCycleOut pivotDutyCycleController;

    private final PositionVoltage pivotPositionVoltageController;

    private final Follower follower;

    public IntakeImpl() {
        pivot = new TalonFX(Ports.Intake.PIVOT);
        Motors.Intake.PIVOT_CONFIG.configure(pivot);

        rollerLeader = new TalonFX(Ports.Intake.ROLLER_LEADER);
        Motors.Intake.ROLLER_LEADER_CONFIG.configure(rollerLeader);

        rollerFollower = new TalonFX(Ports.Intake.ROLLER_FOLLOWER);
        Motors.Intake.ROLLER_FOLLOWER_CONFIG.configure(rollerFollower);
        
        follower = new Follower(Ports.Intake.ROLLER_LEADER, MotorAlignmentValue.Opposed);

        absoluteEncoder = new DutyCycleEncoder(
            Ports.Intake.ABSOLUTE_ENCODER, 
            1.00, 
            Settings.Intake.PIVOT_ANGLE_OFFSET.getRotations()
            ); // TODO: Set Full Range, Verify Expected_Zero wants a Rotation and not Degrees

        rollerDutyCycleController = new DutyCycleOut(0.0);
        pivotDutyCycleController = new DutyCycleOut(0.0);
        pivotPositionVoltageController = new PositionVoltage(0.0);
    }

    /**
     * Returns whether the current Intake angle is within a tolerance of the target angle
     * @return Bool: At Target Angles
     */
    @Override
    public boolean isAtTargetAngle() {
        return Math.abs((getCurrentAngle().getRotations()) - getIntakeState().getTargetAngle().getRotations()) < Settings.Intake.PIVOT_ANGLE_TOLERANCE.getRotations(); 
    }

    /**
     * Gets the Current Intake Pivot Angle using the Relative Encoder built into the Kraken
     * @return Rotation2d: Current Angle (Relative)
     */
    public Rotation2d getRelativeAngle() {
        return Rotation2d.fromRotations(pivot.getPosition().getValueAsDouble());
    }

    /**
     * Gets the Current Intake Pivot Angle using the Absolute Encoder on the Intake
     * @return Rotation2d: Current Angle (Absolute)
     */
    @Override
    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(absoluteEncoder.get());
    }

    @Override
    public void periodic() {

        currentPivotState.position = getCurrentAngle().getRadians();
        currentPivotState.velocity = 0.0;

        TrapezoidProfile profile = new TrapezoidProfile(
            new Constraints(Settings.Intake.ROLLER_MAX_VEL, Settings.Intake.ROLLER_MAX_ACCEL)
        );

        // this is the next step in the profile
        TrapezoidProfile.State nextPivot = profile.calculate(
            Settings.Intake.dT,
            currentPivotState, 
            targetPivotState
        );
    
        // ROLLER CONTROLS
        if (Settings.EnabledSubsystems.INTAKE.getAsBoolean()) {
            pivot.setControl(pivotPositionVoltageController.withPosition(nextPivot.position));
            rollerLeader.setControl(rollerDutyCycleController.withOutput(getIntakeState().getTargetDutyCycle()));
        } 
        else 
        {
            pivot.setControl(pivotDutyCycleController.withOutput(0.0)); 
            rollerLeader.setControl(rollerDutyCycleController.withOutput(0.0));
        }
        rollerFollower.setControl(follower);

        // SMART DASHBOARD
        SmartDashboard.putBoolean("Intake/Pivot/At Target Angle", isAtTargetAngle());

        SmartDashboard.putNumber("Intake/Pivot/Current Angle (deg)", getCurrentAngle().getDegrees());
        SmartDashboard.putNumber("Intake/Pivot/Target Angle (deg)", getIntakeState().getTargetAngle().getDegrees());

        SmartDashboard.putNumber("Intake/Pivot/Angle Error (deg)", Math.abs(getIntakeState().getTargetAngle().getDegrees() - getCurrentAngle().getDegrees()));
        
        if (Settings.DEBUG_MODE) {

            // PIVOT
            SmartDashboard.putString("Intake/Pivot/Current State", getIntakeState().toString());
            SmartDashboard.putNumber("Intake/Pivot/Current Velocity", pivot.getVelocity().getValueAsDouble());
            SmartDashboard.putNumber("Intake/Pivot/Current Angle (Relative Encoder, deg)", getRelativeAngle().getDegrees());

            // ROLLERS
            SmartDashboard.putNumber("Intake/Roller/Duty Cycle Target Speed", getIntakeState().getTargetDutyCycle());
            SmartDashboard.putNumber("Intake/Roller/Current Velocity", rollerLeader.getVelocity().getValueAsDouble());

        }
    }
}
