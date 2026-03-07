package com.stuypulse.robot.commands.climberhopper;

import java.util.Optional;

import com.stuypulse.robot.subsystems.climber.Climber;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberOverrideStop extends Command {
    private final Climber climber;
    
    public ClimberOverrideStop() {
        climber = Climber.getInstance();
    }

    @Override
    public void initialize() {
        climber.setVoltageOverride(Optional.of(0.0));
    }

    @Override
    public void end(boolean interrupted) {
        climber.setVoltageOverride(Optional.empty());
    }
}