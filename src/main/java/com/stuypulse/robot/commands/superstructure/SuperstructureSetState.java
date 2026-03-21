/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;

import edu.wpi.first.wpilibj2.command.Command;

public class SuperstructureSetState extends Command {
    private final Superstructure superstructure;
    private final SuperstructureState state;

    public SuperstructureSetState(SuperstructureState state) {
        superstructure = Superstructure.getInstance();
        this.state = state;

        addRequirements(superstructure);
    }

    @Override 
    public void execute() {
        superstructure.setState(state);
    }

    @Override
    public boolean isFinished() {
        return superstructure.getState() == state;
    }
}