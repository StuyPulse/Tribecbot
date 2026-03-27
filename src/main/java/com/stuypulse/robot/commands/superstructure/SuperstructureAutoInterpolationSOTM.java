/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;

public class SuperstructureAutoInterpolationSOTM extends SuperstructureSetState {
    public SuperstructureAutoInterpolationSOTM() {
        super(SuperstructureState.AUTO_INTERPOLATION_SOTM);
    }
}
