package com.stuypulse.robot.commands.climberhopper;

import com.stuypulse.robot.subsystems.climber.Climber.ClimberState;

public class ClimberStop extends ClimberHopperSetState{
    public ClimberStop() {
        super(ClimberState.CLIMBER_STOP);
    }
}
