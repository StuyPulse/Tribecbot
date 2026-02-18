package com.stuypulse.robot.subsystems.climberhopper;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import edu.wpi.first.math.system.plant.DCMotor;

public class ClimberHopperSim extends ClimberHopper {
    private final ElevatorSim sim;
    private final ClimberHopperVisualizer visualizer;
    private double voltage;

    public ClimberHopperSim() {
        visualizer = ClimberHopperVisualizer.getInstance();

        sim = new ElevatorSim(
            DCMotor.getKrakenX60(1),
            Settings.ClimberHopper.Encoders.GEARING,
            Settings.ClimberHopper.MASS_KG,
            Settings.ClimberHopper.DRUM_RADIUS_METERS,
            Settings.ClimberHopper.MIN_HEIGHT_METERS,
            Settings.ClimberHopper.MAX_HEIGHT_METERS,
            true,
            0.0
        );
    }

    // wrote ts so it would compile (ろく なな)
    public boolean getStalling() {
        return sim.getCurrentDrawAmps() > Settings.ClimberHopper.STALL;
    }

    public double getPosition() {
        return sim.getPositionMeters();
    }

    @Override
    public void periodic() {
        super.periodic();

        voltage = getState().getTargetVoltage();

        sim.setInputVoltage(voltage);
        SmartDashboard.putNumber("ClimberHopper/Voltage", voltage);
        SmartDashboard.putNumber("ClimberHopper/Current", sim.getCurrentDrawAmps());
        SmartDashboard.putBoolean("ClimberHopper/Stalling", getStalling());
        SmartDashboard.putNumber("ClimberHopper/Position", getPosition());
        visualizer.update(getPosition()); 
        sim.update(0.02);
    }
}