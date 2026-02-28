package com.stuypulse.robot.commands.climberhopper;

import java.util.Optional;

import com.stuypulse.robot.subsystems.climberhopper.ClimberHopper;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberOverrideStop extends Command {
    private final ClimberHopper climberHopper;
    
    public ClimberOverrideStop() {
        climberHopper = ClimberHopper.getInstance();
    }

    @Override
    public void initialize() {
        climberHopper.setVoltageOverride(Optional.of(0.0));
    }

    @Override
    public void end(boolean interrupted) {
        climberHopper.setVoltageOverride(Optional.empty());
    }
}