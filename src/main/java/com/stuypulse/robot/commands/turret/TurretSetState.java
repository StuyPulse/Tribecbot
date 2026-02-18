/************************ PROJECT ALPHA *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.turret;

import com.stuypulse.robot.subsystems.turret.Turret;
import com.stuypulse.robot.subsystems.turret.Turret.TurretState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class TurretSetState extends InstantCommand {
     
    private final Turret turret;
    private final TurretState state;

    public TurretSetState(TurretState state) {
        this.turret = Turret.getInstance();
        this.state = state;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.setState(state);
    }
}