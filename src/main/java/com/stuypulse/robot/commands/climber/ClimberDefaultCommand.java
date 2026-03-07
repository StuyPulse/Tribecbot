/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.climber;

import com.stuypulse.robot.subsystems.climber.Climber;
import com.stuypulse.robot.subsystems.climber.Climber.ClimberState;

import edu.wpi.first.wpilibj2.command.Command;


public class ClimberDefaultCommand extends Command {
    private final Climber climber;

    public ClimberDefaultCommand() {
        climber = Climber.getInstance();

        
        addRequirements(climber);
    }

    @Override
    public void execute() {
        if (climber.getState() != ClimberState.CLIMBER_CLIMB) {
            // Set the climber down
            climber.setState(ClimberState.CLIMBER_DOWN);
        }
    }
}