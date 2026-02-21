package com.stuypulse.robot.commands.leds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

import com.stuypulse.robot.constants.Settings.LEDS.LEDState;
import com.stuypulse.robot.subsystems.leds.LEDController;

public class LEDApplyState extends Command {
    protected final LEDController leds;
    protected final Supplier<LEDState> state;

    public LEDApplyState(Supplier<LEDState> state) {
        leds = LEDController.getInstance();
        this.state = state;

        addRequirements(leds);
    }

    public LEDApplyState(LEDState state) {
        this(() -> state);
    }

    @Override
    public void execute() {
        leds.applyState(state.get());
    }
    
}
