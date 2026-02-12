package com.stuypulse.robot.commands.feeder;
import com.stuypulse.robot.subsystems.feeder.Feeder.FeederState;

public class SetFeederStop extends SetFeederState {
    public SetFeederStop() {
        super(FeederState.STOP);
    }
}