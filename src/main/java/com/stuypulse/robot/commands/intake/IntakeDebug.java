package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake.PivotState;

public class IntakeDebug extends IntakeSetState{
    public IntakeDebug() {
        super(PivotState.DEBUG);
    }
    
}
