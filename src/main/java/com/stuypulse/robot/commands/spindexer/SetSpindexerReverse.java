package com.stuypulse.robot.commands.spindexer;
import com.stuypulse.robot.subsystems.spindexer.Spindexer.SpindexerState;

public class SetSpindexerReverse extends SetSpindexerState{
    public SetSpindexerReverse() {
        super(SpindexerState.REVERSE);
    }
}