package com.stuypulse.robot.subsystems.intake;

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
        INTAKE(Settings.Intake.PIVOT_INTAKE_OUTAKE_ANGLE, 1.0), // change later
        OUTAKE(Settings.Intake.PIVOT_INTAKE_OUTAKE_ANGLE, -1.0),
        STOW(Settings.Intake.PIVOT_STOW_ANGLE, 0.0);

        private double targetDutyCycle;
        private Rotation2d targetAngle;
    
        private IntakeState(Rotation2d targetAngle, double targetDutyCycle) {
            this.targetAngle = targetAngle;
            this.targetDutyCycle = targetDutyCycle;
        }

        /**
         * Gets the Target Angle for the Pivot of the Current State of the Intake
         * @return Rotation2d Target Angle
         */
        public Rotation2d getTargetAngle() {
            return targetAngle;
        }

        /**
         * Gets the Target Duty Cycle for the Rollers of the Current State of the Intake
         * @return double Target Duty Cycle
         */
        public double getTargetDutyCycle() {
            return targetDutyCycle;
        }
    }

    public Intake() {
        state = IntakeState.STOW;
    }

    /**
     * Gets the current IntakeState of the Intake
     * @return IntakeState: Current Intake State
     */
    public IntakeState getIntakeState() {
        return state;
    }
    
    /**
     * Sets the Intake State to a new State
     * @param state Desired IntakeState
     * @return Void
     */
    public void setIntakeState(IntakeState state)  {
        this.state = state;
    }

    public abstract boolean isAtTargetAngle();

    public abstract Rotation2d getCurrentAngle();

}
