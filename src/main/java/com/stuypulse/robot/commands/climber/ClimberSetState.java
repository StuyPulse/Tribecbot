/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.climber;

import com.stuypulse.robot.subsystems.climber.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import com.stuypulse.robot.subsystems.climber.Climber.ClimberState;

public class ClimberSetState extends Command {
    private final Climber climber = Climber.getInstance();
    private final ClimberState state;
    
    public ClimberSetState(ClimberState state) {
        this.state = state;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.setState(state);
    }
} 