/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.climberhopper;

import com.stuypulse.robot.subsystems.climber.Climber.ClimberState;

public class ClimberDown extends ClimberHopperSetState {
    public ClimberDown() {
        super(ClimberState.CLIMBER_DOWN);
    }
}