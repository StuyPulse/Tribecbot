package com.stuypulse.robot.commands.spindexer;

import com.stuypulse.robot.subsystems.spindexer.Spindexer.SpindexerState;

public class SetSpindexerStop extends SetSpindexerState {
    public SetSpindexerStop() {
        super(SpindexerState.STOP);
    }
}