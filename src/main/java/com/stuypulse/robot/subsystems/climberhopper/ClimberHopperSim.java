package com.stuypulse.robot.subsystems.climberhopper;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import edu.wpi.first.math.system.plant.DCMotor;

public class ClimberHopperSim extends ClimberHopper {
    private final ElevatorSim sim;
    public ClimberHopperSim() {
        sim = new ElevatorSim(
            DCMotor.getKrakenX60(1),
            Settings.ClimberHopper.Encoders.GEARING,
            Settings.ClimberHopper.MASS_KG,
            Settings.ClimberHopper.Encoders.DRUM_RADIUS_METERS,
            Settings.ClimberHopper.MIN_HEIGHT_METERS,
            Settings.ClimberHopper.MAX_HEIGHT_METERS,
            true,
            0.0
        );
    }

    // wrote ts so it would compile (ろく なな)
    public Boolean getStalling() {
        return sim.getCurrentDrawAmps() > Settings.ClimberHopper.STALL;
    }

    @Override
    public void periodic() {
        super.periodic();

        double voltage = getState().getTargetVoltage();
        
        BStream stalling = BStream.create(() -> sim.getCurrentDrawAmps() > Settings.ClimberHopper.STALL)
            .filtered(new BDebounce.Both(Settings.ClimberHopper.DEBOUNCE));

        if ((getState() == ClimberHopperState.CLIMBER_UP || getState() == ClimberHopperState.HOPPER_UP) && stalling.getAsBoolean()) {
            voltage = Settings.ClimberHopper.RETRACTED_UP;
        }
        else if ((getState() == ClimberHopperState.CLIMBER_DOWN || getState() == ClimberHopperState.HOPPER_DOWN) && stalling.getAsBoolean()) {
            voltage = Settings.ClimberHopper.RETRACTED_DOWN;
        }

        sim.setInputVoltage(voltage);
        SmartDashboard.putNumber("ClimberHopper/voltage", voltage);
        SmartDashboard.putNumber("ClimberHopper/current", sim.getCurrentDrawAmps());
        SmartDashboard.putBoolean("ClimberHopper/stalling", stalling.getAsBoolean());
    }
}