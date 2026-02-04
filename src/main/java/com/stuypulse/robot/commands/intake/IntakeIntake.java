package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;

public class IntakeIntake extends IntakeSetState {
    public IntakeIntake() {
        super(IntakeState.INTAKE);
    }
}
