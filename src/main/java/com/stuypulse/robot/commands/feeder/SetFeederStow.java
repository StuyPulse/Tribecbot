package com.stuypulse.robot.commands.feeder;
import com.stuypulse.robot.subsystems.feeder.Feeder.FeederState;

public class SetFeederStow extends SetFeederState {
    public SetFeederStow() {
        super(FeederState.STOW);
    }
}