/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.turret;

import com.stuypulse.stuylib.input.Gamepad;

import com.stuypulse.robot.subsystems.turret.Turret;
import com.stuypulse.robot.subsystems.turret.Turret.TurretState;

import edu.wpi.first.wpilibj2.command.Command;

public class TurretAnalog extends Command {
    private Gamepad gamepad;
    private Turret turret;
    public TurretAnalog(Gamepad gamepad){
        turret = Turret.getInstance();
        this.gamepad = gamepad;
    }

    @Override 
    public void initialize() {
        super.initialize();
        turret.setState(TurretState.TESTING);
    }
    @Override
    public void execute() {
        super.execute();
        turret.setDriverInput(gamepad);
    }
}