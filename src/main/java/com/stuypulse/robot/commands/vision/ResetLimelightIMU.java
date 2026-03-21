/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.constants.Settings;

public class ResetLimelightIMU extends SetIMUMode {
    public ResetLimelightIMU() {
        super(Settings.Vision.RESET_IMU_INDEX);
    }
}