package com.stuypulse.robot.commands.climberhopper;

import com.stuypulse.robot.subsystems.climberhopper.ClimberHopper.ClimberHopperState;

public class HopperDown extends ClimberHopperSetState {
    public HopperDown() {
        super(ClimberHopperState.HOPPER_DOWN);
    }
}