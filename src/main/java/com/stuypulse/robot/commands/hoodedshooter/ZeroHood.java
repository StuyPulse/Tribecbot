package com.stuypulse.robot.commands.hoodedshooter;

import com.stuypulse.robot.subsystems.hoodedshooter.hood.Hood;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ZeroHood extends InstantCommand{
    private final Hood hood;

    public ZeroHood() {
        hood = Hood.getInstance();

        addRequirements(hood);
    }

    @Override
    public void initialize() {
        hood.zeroHoodEncoder();
        // hood.seedHood();
    }
}
