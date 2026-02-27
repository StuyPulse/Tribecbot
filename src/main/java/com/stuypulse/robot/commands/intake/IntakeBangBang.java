package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake.PivotState;

public class IntakeBangBang extends IntakeSetState {
    public IntakeBangBang() {
        super(PivotState.BANGBANG);
    }
}
