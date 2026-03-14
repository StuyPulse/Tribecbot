package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.subsystems.vision.LimelightVision;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetLimelightBlackList extends InstantCommand {
    private LimelightVision vision;
    private int[] blacklistedArray;
    private String limelightName;

    public SetLimelightBlackList(int[] blacklistedArray, String limelightName) {
        vision = LimelightVision.getInstance();
        this.blacklistedArray = blacklistedArray ;
        this.limelightName = limelightName;
    }

    @Override
    public void initialize() {
        vision.setTagBlacklist(blacklistedArray, limelightName);
    }
    
}