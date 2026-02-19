/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
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