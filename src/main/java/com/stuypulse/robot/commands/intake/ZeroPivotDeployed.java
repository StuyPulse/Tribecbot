/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.intake.Intake.PivotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ZeroPivotDeployed extends InstantCommand {
    private Intake intake;

    public ZeroPivotDeployed() {
        intake = Intake.getInstance();
    }

    @Override
    public void initialize() {
        intake.zeroPivotDeployed();
        intake.setPivotState(PivotState.DEPLOY);
    }
}
