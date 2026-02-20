package com.stuypulse.robot.commands.climberhopper;

import com.stuypulse.robot.subsystems.climberhopper.ClimberHopper.ClimberHopperState;

public class ClimberDown extends ClimberHopperSetState {
    public ClimberDown() {
        super(ClimberHopperState.CLIMBER_DOWN);
    }
}