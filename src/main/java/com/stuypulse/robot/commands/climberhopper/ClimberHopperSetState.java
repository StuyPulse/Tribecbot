/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.climberhopper;

import com.stuypulse.robot.subsystems.climberhopper.*;
import com.stuypulse.robot.subsystems.climberhopper.ClimberHopper.ClimberHopperState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ClimberHopperSetState extends InstantCommand {
    private final ClimberHopper climberHopper = ClimberHopper.getInstance();
    private final ClimberHopperState state;
    
    public ClimberHopperSetState(ClimberHopperState state) {
        this.state = state;
        addRequirements(climberHopper);
    }

    @Override
    public void initialize() {
        climberHopper.setState(state);
    }
} 