package com.stuypulse.robot.commands.feeder;
import com.stuypulse.robot.subsystems.feeder.Feeder.FeederState;

public class SetFeederForward extends SetFeederState {
    public SetFeederForward() {
        super(FeederState.FORWARD);
    }
}