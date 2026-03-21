/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.hood;

import com.stuypulse.stuylib.input.Gamepad;

import com.stuypulse.robot.subsystems.superstructure.hood.Hood;

import edu.wpi.first.wpilibj2.command.Command;

public class HoodAnalog extends Command{
    private final Hood hood;
    private final Gamepad driver;

    public HoodAnalog(Gamepad driver) {
        this.driver = driver;
        hood = Hood.getInstance();

        addRequirements(hood);
    }

    @Override
    public void initialize() {
        hood.setState(Hood.HoodState.ANALOG);
    }

    @Override
    public void execute() {
        hood.hoodAnalogToInput(driver);
        //SmartDashboard.putNumber("Superstructure/Hood/Analog", hood.hoodAnalogToOutput().getDegrees());
    }
}
