package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.subsystems.vision.LimelightVision;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ReleaseObjectPose extends InstantCommand {
    private LimelightVision vision;

    public ReleaseObjectPose() {
        vision = LimelightVision.getInstance();
    }

    @Override
    public void initialize() {
        vision.releasePose();
    }
}