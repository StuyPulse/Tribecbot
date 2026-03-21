/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake.PivotState;
import com.stuypulse.robot.subsystems.intake.Intake.RollerState;

public class IntakeDeploy extends IntakeSetState {
    public IntakeDeploy() {
        super(PivotState.DEPLOYED, RollerState.INTAKE);
    }
}
