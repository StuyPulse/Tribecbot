
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.leds;


import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.ResourceBundle.Control;
import java.util.function.Supplier;

import com.ctre.phoenix6.controls.ControlRequest;
import com.stuypulse.robot.subsystems.leds.LEDController;

public class LEDApplyPattern extends Command {

    protected final LEDController leds;
    protected final Supplier<ControlRequest> pattern;

    public LEDApplyPattern(Supplier<ControlRequest> pattern) {
        leds = LEDController.getInstance();
        this.pattern = pattern;

        addRequirements(leds);
    }

    public LEDApplyPattern(ControlRequest pattern) {
        this(() -> pattern);
    }

    @Override
    public void execute() {
        leds.applyPattern(pattern.get());
    }

}
