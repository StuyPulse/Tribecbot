package com.stuypulse.robot.subsystems.climberhopper;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ClimberHopper extends SubsystemBase {
    private static final ClimberHopper instance;
    
    static {
        instance = new ClimberHopperImpl();
    }

    public static ClimberHopper getInstance() {
        return instance;
    }

    public enum ClimberHopperState {
        CLIMBER_UP(Settings.ClimberHopper.CLIMBER_UP),
        CLIMBER_DOWN(Settings.ClimberHopper.CLIMBER_DOWN),
        HOPPER_UP(Settings.ClimberHopper.HOPPER_UP),
        HOPPER_DOWN(Settings.ClimberHopper.HOPPER_DOWN),
        HOLDING_UP(Settings.ClimberHopper.EXTENDED),
        HOLDING_DOWN(Settings.ClimberHopper.RETRACTED);
    
        private double targetVoltage;
        
        private ClimberHopperState(double targetVoltage) {
            this.targetVoltage = targetVoltage;
        }
        
        public double getTargetVoltage() {
            return targetVoltage;
        }

    }
    
    protected ClimberHopperState state;

    public ClimberHopper() {
        this.state = ClimberHopperState.CLIMBER_UP;
    }
        
    public ClimberHopperState getState() {
        return state;
    }

    public void setState(ClimberHopperState state) {
        this.state = state;
    }

    public abstract boolean getStalling();
    public abstract double getPosition();

    @Override
    public void periodic() {
        SmartDashboard.putString("ClimberHopper/State", getState().toString());
    }
}