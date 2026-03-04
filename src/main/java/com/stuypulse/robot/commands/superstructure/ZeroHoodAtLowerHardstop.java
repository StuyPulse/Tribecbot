package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.subsystems.hoodedshooter.hood.Hood;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ZeroHoodAtLowerHardstop extends InstantCommand{
    private final Hood hood;

    public ZeroHoodAtLowerHardstop() {
        hood = Hood.getInstance();

        addRequirements(hood);
    }

    @Override
    public void initialize() {
        hood.zeroHoodEncoderAtLowerHardstop();
        hood.seedHood();
    }
}
