package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.Robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Intake extends SubsystemBase {
    private static final Intake instance;
    private IntakeState state;

    static {
        //if (Robot.isReal()) {
            instance = new IntakeImpl();
        //}
    }

    public static Intake getInstance() {
        return instance;
    }

    public enum IntakeState { 
        INTAKE(new Rotation2d(), 1.0), // change later
        OUTAKE(new Rotation2d(), -1.0),
        STOW(new Rotation2d(), 0.0);

        private double targetDutyCycle;
        private Rotation2d targetAngle;
        

        private IntakeState(Rotation2d targetAngle, double targetDutyCycle) {
            this.targetAngle = targetAngle;
            this.targetDutyCycle = targetDutyCycle;
        }

        public Rotation2d getTargetAngle() {
            return targetAngle;
        }

        public double getTargetDutyCycle() {
            return targetDutyCycle;
        }
    }

    public IntakeState getIntakeState() {
        return state;
    }
    
    public void setIntakeState(IntakeState state)  {
        this.state = state;
    }

    public abstract boolean isAtTargetAngle();

    public abstract Rotation2d getCurrentAngle();

    public abstract Rotation2d getCurrentAngleFromAbsoluteEncoder();

    @Override
    public void periodic() {

    }
}
