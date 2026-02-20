package com.stuypulse.robot.commands.climberhopper;

import com.stuypulse.robot.subsystems.climberhopper.ClimberHopper.ClimberHopperState;

public class ClimberUp extends ClimberHopperSetState {
    public ClimberUp() {
        super(ClimberHopperState.CLIMBER_UP);
    }
}