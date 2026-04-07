package com.stuypulse.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class BlackListAllTagsForAllCameras extends ParallelCommandGroup{
    public BlackListAllTagsForAllCameras() {
        addCommands(
            new BlacklistAllTags("limelight-left"),
            new BlacklistAllTags("limelight-right"),
            new BlacklistAllTags("limelight-back")
        );
    }
}
