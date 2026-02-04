package com.stuypulse.robot.subsystems.intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;

public class IntakeImpl extends Intake {
    private final TalonFX pivot;
    private final TalonFX rollerMaster;
    private final TalonFX rollerSlave;
    private final DutyCycleEncoder absoluteEncoder;

    public IntakeImpl() {
        pivot = new TalonFX(Ports.Intake.PIVOT);
        Motors.Intake.PIVOT_CONFIG.configure(pivot);

        rollerMaster = new TalonFX(Ports.Intake.ROLLER_MASTER);
        Motors.Intake.ROLLER_MASTER_CONFIG.configure(rollerMaster);

        rollerSlave = new TalonFX(Ports.Intake.ROLLER_SLAVE);
        Motors.Intake.ROLLER_SLAVE_CONFIG.configure(rollerSlave);

        absoluteEncoder = new DutyCycleEncoder(Ports.Intake.ABSOLUTE_ENCODER);
        absoluteEncoder.setInverted(false);

        pivot.setControl(new PositionVoltage(getIntakeState().getTargetAngle().getDegrees()));
        rollerMaster.setControl(new DutyCycleOut(getIntakeState().getTargetDutyCycle()));
        rollerSlave.setControl(new Follower(Ports.Intake.ROLLER_MASTER, MotorAlignmentValue.Opposed));
    }

    @Override
    public boolean isAtTargetAngle() {
        return Math.abs(getCurrentAngle().getRadians() - getIntakeState().getTargetAngle().getRadians()) < Settings.Intake.PIVOT_ANGLE_TOLERANCE; 
    }

    @Override
    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(pivot.getPosition().getValueAsDouble());
    }

    @Override
    public Rotation2d getCurrentAngleFromAbsoluteEncoder() {
        double angleRotations = absoluteEncoder.get() - Settings.Intake.ANGLE_OFFSET.getRotations();
        return Rotation2d.fromRotations(angleRotations > Settings.Intake.MAXIMUM_ANGLE.getRotations() + Units.degreesToRotations(10)
            ? angleRotations - 1
            : angleRotations);
    }

    @Override
    public void periodic() {
        // PIVOT
        SmartDashboard.putString("Intake/Pivot/Current State", getIntakeState().toString());
        SmartDashboard.putBoolean("Intake/Pivot/At Target Angle", isAtTargetAngle());
        SmartDashboard.putNumber("Intake/Pivot/Current Velocity", pivot.getVelocity().getValueAsDouble());

        // ROLLERG
        SmartDashboard.putNumber("Intake/Roller/Duty Cycle Target Speed", getIntakeState().getTargetDutyCycle());
        SmartDashboard.putNumber("Intake/Roller/Current Velocity", rollerMaster.getVelocity().getValueAsDouble());
    }
}
