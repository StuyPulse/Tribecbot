package com.stuypulse.robot.commands.hood;

import com.stuypulse.robot.subsystems.superstructure.hood.Hood;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class HomingRoutineLower extends InstantCommand {
    private final Hood hood;

    public HomingRoutineLower() {
        hood = Hood.getInstance();

        addRequirements(hood);
    }

    @Override
    public void initialize() {
        hood.setState(Hood.HoodState.HOMING_LOWER);
    }
    
}
