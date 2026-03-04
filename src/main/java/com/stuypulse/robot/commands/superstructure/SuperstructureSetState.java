/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.subsystems.Superstructure.Superstructure;
import com.stuypulse.robot.subsystems.Superstructure.Superstructure.SuperstructureState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SuperstructureSetState extends InstantCommand {
    private final Superstructure Superstructure;
    private final SuperstructureState state;

    public SuperstructureSetState(SuperstructureState state) {
        Superstructure = Superstructure.getInstance();
        this.state = state;

        addRequirements(Superstructure);
    }

    @Override 
    public void execute() {
        Superstructure.setState(state);
    }
}