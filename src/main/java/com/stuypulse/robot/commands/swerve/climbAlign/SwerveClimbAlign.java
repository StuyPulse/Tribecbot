/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.swerve.climbAlign;

import java.io.Console;

import com.stuypulse.robot.constants.DriverConstants;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;

public class SwerveClimbAlign extends ConditionalCommand {

    public SwerveClimbAlign(Gamepad driver) {
        super(new SwerveClimbAlignTop(driver), new SwerveClimbAlignBot(driver), () -> Field.closerToTop());
    }

    public SwerveClimbAlign() {
        super(new SwerveClimbAlignTop(null), new SwerveClimbAlignBot(null), () -> Field.closerToTop());
    }

    
}