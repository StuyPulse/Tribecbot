package com.stuypulse.robot.commands.feeder;

import com.stuypulse.robot.subsystems.feeder.Feeder;
import com.stuypulse.robot.subsystems.feeder.Feeder.FeederState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetFeederState extends InstantCommand{
    private final Feeder feeder;
    private FeederState state;

    public SetFeederState(FeederState state) {
        this.feeder = Feeder.getInstance();
        this.state = state;
        addRequirements(feeder);
    }

    @Override
    public void initialize() {
        feeder.setFeederState(state);
    }
}



