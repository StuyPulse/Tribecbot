package com.stuypulse.robot.commands.feeder;
import com.stuypulse.robot.subsystems.feeder.Feeder.FeederState;

public class FeederReverse extends SetFeederState {
    public FeederReverse() {
        super(FeederState.REVERSE);
    }
}