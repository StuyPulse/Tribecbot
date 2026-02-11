package com.stuypulse.robot.subsystems.feeder;

import java.util.Optional;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public abstract class Feeder extends SubsystemBase {
    private static final Feeder instance;

    static {
        instance = new FeederImpl();
    }

    public static Feeder getInstance() {
        return instance;
    }

    public enum FeederState {
        STOW(Settings.Feeder.STOW_RPM),
        FORWARD(Settings.Feeder.FORWARD_RPM),
        REVERSE(Settings.Feeder.REVERSE_RPM),
        STOP(0.0);

        private double targetRPM;

        private FeederState(double targetRPM) {
            this.targetRPM = targetRPM;
        }

        public double getTargetRPM() {
            return this.targetRPM;
        }
    }

    private FeederState state;

    public Feeder() {
        state = FeederState.STOP;
    }

    /**
     * @return target RPM based on current state
     */
    public double getTargetRPM() {
        return state.getTargetRPM();
    }

    /**
     * @return current feeder state
     */
    public FeederState getFeederState() {
        return state;
    }

    /**
     * @param state to set new feeder state
     */
    public void setFeederState(FeederState state) {
        this.state = state;
    }

    public abstract SysIdRoutine getSysIdRoutine();

    public abstract double getVoltageOverride();

    public abstract void setVoltageOverride(Optional<Double> voltage);

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putString("Feeder/State", getFeederState().toString());
        SmartDashboard.putNumber("Feeder/Speed", getTargetRPM());
    }
}
