package com.stuypulse.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase {
    private static final Spindexer instance;
    private SpindexerState state;

    static {
        instance = new SpindexerImpl();
    }

    public static Spindexer getInstance() {
        return instance;
    }

    public enum SpindexerState {
        STOP();

        private double targetDutyCycle;
        
        private SpindexerState() {

        }
    }

    @Override
    public void periodic() {

    }
}
