package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;

public class IntakeIntake extends IntakeSetState {
    /**
     * Sets the State of the Intake to Intaking
     */
    public IntakeIntake() {
        super(IntakeState.INTAKE);
    }
}
