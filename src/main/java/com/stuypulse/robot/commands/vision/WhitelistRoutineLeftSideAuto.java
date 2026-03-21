package com.stuypulse.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class WhitelistRoutineLeftSideAuto extends ParallelCommandGroup {
    public WhitelistRoutineLeftSideAuto() {
        addCommands(
            new WhitelistTowerTags("limelight-right"),
            new BlacklistAllTags("limelight-left"),
            new BlacklistAllTags("limelight-back")
        );
    }
}