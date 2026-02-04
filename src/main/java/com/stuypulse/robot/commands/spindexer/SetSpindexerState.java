package com.stuypulse.robot.commands.spindexer;

import com.stuypulse.robot.subsystems.spindexer.Spindexer;
import com.stuypulse.robot.subsystems.spindexer.Spindexer.SpindexerState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetSpindexerState extends InstantCommand {
    private final Spindexer spindexer;
    private SpindexerState state;

    public SetSpindexerState(SpindexerState state) {
        this.spindexer = Spindexer.getInstance();
        this.state = state;
        addRequirements(spindexer);
    }

    @Override
    public void initialize() {
        spindexer.setSpindexerState(state);
    }
}