
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.leds;


import java.util.function.Supplier;

import com.stuypulse.robot.subsystems.leds.LEDController;
import com.stuypulse.robot.subsystems.leds.LEDController.LedState;

import edu.wpi.first.wpilibj2.command.Command;

public class LEDApplyState extends Command {
    protected final LEDController leds;
    protected final Supplier<LedState> state;

    public LEDApplyState(Supplier<LedState> state) {
        leds = LEDController.getInstance();
        this.state = state;

        addRequirements(leds);
    }

    public LEDApplyState(LedState state) {
        this(() -> state);
    }

    @Override
    public void execute() {
        leds.changeState(state.get());
    }

}
