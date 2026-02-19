/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.turret;

import com.stuypulse.robot.subsystems.turret.Turret;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class TurretSeed extends InstantCommand {
    private Turret turret;

    public TurretSeed() {
        turret = Turret.getInstance();

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.seedTurret();
    }
}