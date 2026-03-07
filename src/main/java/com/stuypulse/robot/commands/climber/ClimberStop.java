package com.stuypulse.robot.commands.climber;

import com.stuypulse.robot.subsystems.climber.Climber.ClimberState;

public class ClimberStop extends ClimberSetState{
    public ClimberStop() {
        super(ClimberState.CLIMBER_STOP);
    }
}
