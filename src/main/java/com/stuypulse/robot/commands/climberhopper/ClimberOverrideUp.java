package com.stuypulse.robot.commands.climberhopper;

import java.util.Optional;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.climberhopper.ClimberHopper;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberOverrideUp extends Command {
   private final ClimberHopper climberHopper;
    
    public ClimberOverrideUp() {
        climberHopper = ClimberHopper.getInstance();
    }

    @Override
    public void initialize() {
        climberHopper.setVoltageOverride(Optional.of(Settings.ClimberHopper.MOTOR_VOLTAGE));
    }

    @Override
    public void end(boolean interrupted) {
        climberHopper.setVoltageOverride(Optional.empty());
    }
}
