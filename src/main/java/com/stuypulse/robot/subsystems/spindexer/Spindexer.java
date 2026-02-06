package com.stuypulse.robot.subsystems.spindexer;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Spindexer extends SubsystemBase {
    private static final Spindexer instance;
    private SpindexerState spindexerState;
    private double rpm;

    static {
        instance = new SpindexerImpl();
    }

    public static Spindexer getInstance() {
        return instance;
    }

    public enum SpindexerState {
        STOP,
        DYNAMIC,
        FORWARD,
        REVERSE;
    }

    protected Spindexer() {
        spindexerState = SpindexerState.STOP;
    }

    public SpindexerState getSpindexerState() {
        return spindexerState;
    }

    public void setSpindexerState(SpindexerState state) {
        this.spindexerState = state;
    }

    public void setTargetRPM(double rpm) {
        this.rpm = rpm;
    }

    public double getTargetRPM() {
        return switch (getSpindexerState()) {
            case STOP -> 0;
            case DYNAMIC -> getRPMBasedOnDistance();
            case FORWARD -> Settings.Spindexer.FORWARD_RPM;
            case REVERSE -> Settings.Spindexer.REVERSE_RPM;
        };
    }

    public abstract double getRPMBasedOnDistance(); // implement in SpindexerImpl

    @Override
    public void periodic() {
        SmartDashboard.putString("Spindexer/State", getSpindexerState().toString());
        SmartDashboard.putNumber("Spindexer/Target RPM", getTargetRPM());
    }
}