package com.stuypulse.robot.commands.spindexer;

import com.stuypulse.robot.subsystems.spindexer.Spindexer.SpindexerState;

public class SetSpindexerDynamic extends SetSpindexerState {
    public SetSpindexerDynamic() {
        super(SpindexerState.DYNAMIC);
    }
}