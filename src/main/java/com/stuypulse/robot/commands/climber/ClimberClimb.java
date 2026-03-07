package com.stuypulse.robot.commands.climber;

import com.stuypulse.robot.subsystems.climber.Climber.ClimberState;

public class ClimberClimb extends ClimberSetState {
    public ClimberClimb() {
        super(ClimberState.CLIMBER_CLIMB);
    }
}
