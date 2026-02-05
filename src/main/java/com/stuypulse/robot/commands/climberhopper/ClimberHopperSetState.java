package com.stuypulse.robot.commands.climberhopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.stuypulse.robot.subsystems.climberhopper.*;
import com.stuypulse.robot.subsystems.climberhopper.ClimberHopper.ClimberHopperState;

public class ClimberHopperSetState extends InstantCommand {
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