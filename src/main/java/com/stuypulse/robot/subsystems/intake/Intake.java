package com.stuypulse.robot.subsystems.intake;

import java.util.function.Supplier;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Intake extends SubsystemBase {
    private static final Intake instance;
    private IntakeState state;

    static {
        if (Robot.isReal()) {
            instance = new IntakeImpl();
        }
        else {
            instance = new IntakeSim();
        }
    }

    public static Intake getInstance() {
        return instance;
    }

    public enum IntakeState { 
        INTAKE(() -> Rotation2d.fromDegrees(Settings.Intake.PIVOT_INTAKE_OUTAKE_ANGLE.get()), 1.0), // change later
        OUTAKE(() -> Rotation2d.fromDegrees(Settings.Intake.PIVOT_INTAKE_OUTAKE_ANGLE.get()), -1.0),
        STOW(() -> Rotation2d.fromDegrees(Settings.Intake.PIVOT_STOW_ANGLE.get()), 0.0);

        private double targetDutyCycle;
        private Supplier<Rotation2d> targetAngle;
        

        private IntakeState(Supplier<Rotation2d> targetAngle, double targetDutyCycle) {
            this.targetAngle = targetAngle;
            this.targetDutyCycle = targetDutyCycle;
        }

        public Supplier<Rotation2d> getTargetAngle() {
            return targetAngle;
        }

        public double getTargetDutyCycle() {
            return targetDutyCycle;
        }
    }

    public Intake() {
        state = IntakeState.STOW;
    }

    public IntakeState getIntakeState() {
        return state;
    }
    
    public void setIntakeState(IntakeState state)  {
        this.state = state;
    }

    public abstract boolean isAtTargetAngle();

    public abstract Rotation2d getCurrentAngle();
    
}
