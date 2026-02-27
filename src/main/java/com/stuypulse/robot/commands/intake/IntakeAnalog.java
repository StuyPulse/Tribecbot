/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.intake;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.intake.Intake.PivotState;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeAnalog extends Command {
    private Gamepad gamepad;
    private Intake intake;
    public IntakeAnalog(Gamepad gamepad){
        intake = Intake.getInstance();
        this.gamepad = gamepad;
    }

    @Override 
    public void initialize() {
        super.initialize();
        intake.setPivotState(PivotState.ANALOG);
    }
    @Override
    public void execute() {
        super.execute();
        intake.setDriverInput(gamepad);
    }
}