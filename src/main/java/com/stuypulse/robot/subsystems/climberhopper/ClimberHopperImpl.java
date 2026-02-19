package com.stuypulse.robot.subsystems.climberhopper;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.stuypulse.robot.constants.Constants;
//import com.stuypulse.stuylib.streams.booleans.filters.BFilter;
import com.stuypulse.robot.constants.Motors;


public class ClimberHopperImpl extends ClimberHopper {
    private final TalonFX motor;
    private final BStream stalling;
    private double voltage;

    public ClimberHopperImpl() {
        super();
        motor = new TalonFX(Ports.ClimberHopper.CLIMBER_HOPPER);
        Motors.ClimberHopper.climberHopperMotor.configure(motor);
        motor.setPosition(0); // hopper all the way down according to le henry
        stalling = BStream.create(() -> motor.getStatorCurrent().getValueAsDouble() > Settings.ClimberHopper.STALL)
            .filtered(new BDebounce.Both(Settings.ClimberHopper.DEBOUNCE));
    }

    @Override 
    public boolean getStalling() {
        return stalling.getAsBoolean();
    }

    public double getPosition() { // TODO: convert motor encoder position to meters somehow
        return this.motor.getPosition().getValueAsDouble() * Constants.ClimberHopper.Encoders.POSITION_CONVERSION_FACTOR;
    }

    @Override
    public void periodic() {
        voltage = getState().getTargetVoltage();

        motor.setVoltage(voltage);
        SmartDashboard.putNumber("ClimberHopper/Voltage", voltage);
        SmartDashboard.putNumber("ClimberHopper/Current", motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("ClimberHopper/Stalling", getStalling());

    }
}