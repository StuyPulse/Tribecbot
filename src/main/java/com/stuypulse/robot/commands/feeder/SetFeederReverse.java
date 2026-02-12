package com.stuypulse.robot.commands.feeder;
import com.stuypulse.robot.subsystems.feeder.Feeder.FeederState;

public class SetFeederReverse extends SetFeederState {
    public SetFeederReverse() {
        super(FeederState.REVERSE);
    }
}