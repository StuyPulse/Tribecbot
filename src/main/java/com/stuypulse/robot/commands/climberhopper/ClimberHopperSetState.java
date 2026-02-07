package com.stuypulse.robot.commands.climberhopper;

import edu.wpi.first.wpilibj2.command.Command;

import com.stuypulse.robot.subsystems.climberhopper.*;
import com.stuypulse.robot.subsystems.climberhopper.ClimberHopper.ClimberHopperState;

public class ClimberHopperSetState extends Command {
    private final ClimberHopper climberHopper = ClimberHopper.getInstance();
    private final ClimberHopperState state;
    
    public ClimberHopperSetState(ClimberHopperState state) {
        this.state = state;
        addRequirements(climberHopper);
    }

    @Override
    public void initialize() {
        climberHopper.setState(state);
    }
} 