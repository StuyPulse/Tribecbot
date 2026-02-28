/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.hoodedshooter;

import com.stuypulse.stuylib.network.SmartBoolean;

public class HoodedShooterSim extends HoodedShooter {

    private final SmartBoolean bothAtTolerance;

    protected HoodedShooterSim() {
        super();
        bothAtTolerance = new SmartBoolean("HoodedShooter/Both At Tolerance", false);
    }

    @Override
    public boolean bothAtTolerance() {
        return bothAtTolerance.get();
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}