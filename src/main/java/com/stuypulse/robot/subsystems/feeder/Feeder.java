package com.stuypulse.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
    private static final Feeder instance;
    private FeederState state;

    static {
        instance = new FeederImpl();
    }

    public static Feeder getInstance() {
        return instance;
    }

    public enum FeederState {
        STOW();

        private double targetDutyCycle;
        
        private FeederState() {

        }
    }

    @Override
    public void periodic() {

    }
}
