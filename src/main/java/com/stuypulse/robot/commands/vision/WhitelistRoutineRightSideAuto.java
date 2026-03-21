package com.stuypulse.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class WhitelistRoutineRightSideAuto extends ParallelCommandGroup {
    public WhitelistRoutineRightSideAuto() {
        addCommands(
            new WhitelistOutpostTags("limelight-left"),
            new BlacklistAllTags("limelight-right"),
            new BlacklistAllTags("limelight-back")
        );
    }
}