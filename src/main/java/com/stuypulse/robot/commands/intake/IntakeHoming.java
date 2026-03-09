package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake.PivotState;

public class IntakeHoming extends IntakeSetState {

    public IntakeHoming() { //TODO: ensure this works/overrides the Intake Deploy w/o conflicts
        super(PivotState.HOMING);
    }
    
}
