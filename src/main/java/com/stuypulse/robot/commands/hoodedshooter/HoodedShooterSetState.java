/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.hoodedshooter;

import com.stuypulse.robot.subsystems.hoodedshooter.HoodedShooter;
import com.stuypulse.robot.subsystems.hoodedshooter.HoodedShooter.HoodedShooterState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class HoodedShooterSetState extends InstantCommand {
    private final HoodedShooter hoodedshooter;
    private final HoodedShooterState state;

    public HoodedShooterSetState(HoodedShooterState state) {
        hoodedshooter = HoodedShooter.getInstance();
        this.state = state;

        addRequirements(hoodedshooter);
    }

    @Override 
    public void execute() {
        hoodedshooter.setState(state);
    }
}