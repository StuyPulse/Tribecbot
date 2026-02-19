package com.stuypulse.robot.commands.feeder;
import com.stuypulse.robot.subsystems.feeder.Feeder.FeederState;

public class FeederStop extends SetFeederState {
    public FeederStop() {
        super(FeederState.STOP);
    }
}