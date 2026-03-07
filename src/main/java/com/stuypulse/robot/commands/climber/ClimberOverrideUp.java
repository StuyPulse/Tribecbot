package com.stuypulse.robot.commands.climber;

import java.util.Optional;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.climber.Climber;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberOverrideUp extends Command {
   private final Climber climber;
    
    public ClimberOverrideUp() {
        climber = Climber.getInstance();
    }

    @Override
    public void initialize() {
        climber.setVoltageOverride(Optional.of(Settings.Climber.MOTOR_VOLTAGE));
    }

    @Override
    public void end(boolean interrupted) {
        climber.setVoltageOverride(Optional.empty());
    }
}
