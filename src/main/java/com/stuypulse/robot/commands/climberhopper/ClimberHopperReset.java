package com.stuypulse.robot.commands.climberhopper;

import com.stuypulse.robot.subsystems.climberhopper.ClimberHopper;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberHopperReset extends Command {
    private final ClimberHopper climberHopper;
    
    public ClimberHopperReset() {
        climberHopper = ClimberHopper.getInstance();
    }

    @Override
    public void execute() {
        
    }
}
