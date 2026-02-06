package com.stuypulse.robot.subsystems.climberhopper;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import com.stuypulse.stuylib.streams.booleans.filters.BFilter;
import com.stuypulse.robot.constants.Motors;


public class ClimberHopperImpl extends ClimberHopper {
    private TalonFX motor;
    private BStream stalling;
    private double voltage;

    public ClimberHopperImpl() {
        super();
        motor = new TalonFX(Ports.ClimberHopper.CLIMBER_HOPPER);
        Motors.ClimberHopper.climberHopperMotor.configure(motor);
    }

    @Override
    public boolean getStalling() {
        return stalling.getAsBoolean();
    }

    @Override
    public void periodic() {
                
        stalling = BStream.create(() -> motor.getSupplyCurrent().getValueAsDouble() > Settings.ClimberHopper.STALL)
            .filtered(new BDebounce.Both(Settings.ClimberHopper.DEBOUNCE));

        boolean isUp = (getState() == ClimberHopperState.CLIMBER_UP || getState() == ClimberHopperState.HOPPER_UP);
        boolean isDown = (getState() == ClimberHopperState.CLIMBER_DOWN || getState() == ClimberHopperState.HOPPER_DOWN);

        if (isUp && stalling.getAsBoolean()) {
            setState(ClimberHopperState.HOLDING_UP);
        }
        else if (isDown && stalling.getAsBoolean()) {
            setState(ClimberHopperState.HOLDING_DOWN);
        }
        
        voltage = getState().getTargetVoltage();

        motor.setVoltage(voltage);
        SmartDashboard.putNumber("ClimberHopper/voltage", voltage);
        SmartDashboard.putNumber("ClimberHopper/current", motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("ClimberHopper/stalling", stalling.getAsBoolean());

    }
}