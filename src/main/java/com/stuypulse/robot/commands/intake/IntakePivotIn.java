package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;

public class IntakePivotIn extends IntakeSetState{

    public IntakePivotIn() {
        super(IntakeState.PIVOTIN);
    }
    
}
