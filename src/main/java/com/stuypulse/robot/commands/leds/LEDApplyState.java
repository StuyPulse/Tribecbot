
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.leds;


import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.ResourceBundle.Control;
import java.util.function.Supplier;

import com.ctre.phoenix6.controls.ControlRequest;
import com.stuypulse.robot.subsystems.leds.LEDController;
import com.stuypulse.robot.subsystems.leds.LEDController.LedState;

public class LEDApplyState extends InstantCommand {
    //This will not work as default command will override it. Either make the cached class the same as this OR (better solution) have a boolean that tells you if it is manually applied or not and if it is then default command dont change
    protected final LEDController leds;
    protected final LedState state;

    public LEDApplyState(LedState state) {
        leds = LEDController.getInstance();
        this.state = state;

        addRequirements(leds);
    }

    @Override
    public void execute() {
        leds.changeState(state);
    }

}
