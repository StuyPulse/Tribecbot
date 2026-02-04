package com.stuypulse.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private static final Climber instance;
    private ClimberState state;

    static {
        instance = new ClimberImpl();
    }

    public static Climber getInstance() {
        return instance;
    }

    public enum ClimberState {
        STOW();

        private double targetHeight;
        
        private ClimberState() {

        }
    }

    @Override
    public void periodic() {

    }
}
