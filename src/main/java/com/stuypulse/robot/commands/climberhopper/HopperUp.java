package com.stuypulse.robot.commands.climberhopper;

import com.stuypulse.robot.subsystems.climberhopper.ClimberHopper.ClimberHopperState;

public class HopperUp extends ClimberHopperSetState {
    public HopperUp() {
        super(ClimberHopperState.HOPPER_UP);
    }
}