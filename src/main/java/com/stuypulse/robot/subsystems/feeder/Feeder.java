package com.stuypulse.robot.subsystems.feeder;

import java.util.Optional;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public abstract class Feeder extends SubsystemBase {
    private static final Feeder instance;
    private FeederState state;

    static {
        instance = new FeederImpl();
    }

    public static Feeder getInstance() {
        return instance;
    }

    public enum FeederState {
        STOP,
        FORWARD,
        REVERSE;
    }

    public Feeder() {
        state = FeederState.STOP;
    }

    public double getTargetRPM() {
        return switch (getState()) {
            case STOP -> 0;
            case FORWARD -> Settings.Feeder.FORWARD_RPM;
            case REVERSE -> Settings.Feeder.REVERSE_RPM;

        };
    }

    public FeederState getState() {
        return state;
    }

    public void setState(FeederState state) {
        this.state = state;
    }

    public boolean atTolerance() {
        double diff = Math.abs(getTargetRPM() - getCurrentRPM());
        return diff < Settings.Feeder.RPM_TOLERANCE;
    }

    public abstract double getCurrentRPM();

    public abstract SysIdRoutine getSysIdRoutine();
    public abstract void setVoltageOverride(Optional<Double> voltage);

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putString("Feeder/State", getState().toString());
        SmartDashboard.putString("States/Feeder", getState().toString());

        SmartDashboard.putNumber("Feeder/Target RPM", getTargetRPM());
        SmartDashboard.putNumber("Feeder/Current RPM", getCurrentRPM());
    }
}
