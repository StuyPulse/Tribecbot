package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;

public class IntakeOutake extends IntakeSetState {
    /**
     * Sets the State of the Intake to Outaking
     */
    public IntakeOutake() {
        super(IntakeState.OUTAKE);
    }
}