package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;

public class IntakePivotOut extends IntakeSetState{
    public IntakePivotOut() {
        super(IntakeState.PIVOTOUT);
    }
    
}
