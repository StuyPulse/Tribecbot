package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;

public class IntakeOutake extends IntakeSetState {
    public IntakeOutake() {
        super(IntakeState.OUTAKE);
    }
}