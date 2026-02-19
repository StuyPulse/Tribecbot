package com.stuypulse.robot.commands.spindexer;
import com.stuypulse.robot.subsystems.spindexer.Spindexer.SpindexerState;

public class SpindexerReverse extends SetSpindexerState{
    public SpindexerReverse() {
        super(SpindexerState.REVERSE);
    }
}