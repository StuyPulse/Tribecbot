package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;

public class IntakeStow extends IntakeSetState {
    public IntakeStow() {
        super(IntakeState.STOW);
    }
}
