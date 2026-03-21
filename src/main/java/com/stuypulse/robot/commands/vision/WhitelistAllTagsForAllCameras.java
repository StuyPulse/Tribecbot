package com.stuypulse.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class WhitelistAllTagsForAllCameras extends ParallelCommandGroup {
    public WhitelistAllTagsForAllCameras() {
        addCommands(
            new WhitelistAllTags("limelight-left"),
            new WhitelistAllTags("limelight-right"),
            new WhitelistAllTags("limelight-back")
        );
    }

}
