package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.subsystems.vision.LimelightVision;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetIMUMode extends InstantCommand {
    private final LimelightVision vision;
    private final int index;

    public SetIMUMode(int IMUIndex) {
        vision = LimelightVision.getInstance();
        index = IMUIndex;
    }

    @Override
    public void initialize() {
        super.initialize();
        vision.setIMUMode(index);
    }
}
