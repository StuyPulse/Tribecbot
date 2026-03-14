package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.constants.Field;
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

    public SetLimelightBlackList(String limelightName) {
        vision = LimelightVision.getInstance();
        this.blacklistedArray = Field.ALL_TAGS;
        this.limelightName = limelightName;
    }

    @Override
    public void initialize() {
        vision.setTagWhitelist(new int[]{}, limelightName);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}