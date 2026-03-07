package com.stuypulse.robot.commands.climberhopper;

import com.stuypulse.robot.subsystems.climberhopper.ClimberHopper.ClimberHopperState;

public class ClimberStop extends ClimberHopperSetState{
    public ClimberStop() {
        super(ClimberHopperState.STOP);
    }
}
