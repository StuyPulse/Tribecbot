package com.stuypulse.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private static final Intake instance;
    private IntakeState state;

    static {
        instance = new IntakeImpl();
    }

    public static Intake getInstance() {
        return instance;
    }

    public enum IntakeState {
        STOW();

        private double targetDutyCycle;
        private Rotation2d targetAngle;
        
        private IntakeState() {

        }
    }

    @Override
    public void periodic() {

    }
}
