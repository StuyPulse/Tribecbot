package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;

public class IntakeStow extends IntakeSetState {
    /**
     * Sets the State of the Intake to Stowing
     */
    public IntakeStow() {
        super(IntakeState.STOW);
    }
}
