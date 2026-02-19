package com.stuypulse.robot.commands.spindexer;

import com.stuypulse.robot.subsystems.spindexer.Spindexer.SpindexerState;

public class SpindexerDynamic extends SetSpindexerState {
    public SpindexerDynamic() {
        super(SpindexerState.DYNAMIC);
    }
}