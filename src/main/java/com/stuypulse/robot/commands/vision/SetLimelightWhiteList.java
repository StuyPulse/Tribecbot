package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.vision.LimelightVision;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetLimelightWhiteList extends InstantCommand {
    private LimelightVision vision;
    private int[] whitelistedArray;
    private String limelightName;

    public SetLimelightWhiteList(int[] whitelistedArray, String limelightName) {
        vision = LimelightVision.getInstance();
        this.whitelistedArray = whitelistedArray;
        this.limelightName = limelightName;
    }

    public SetLimelightWhiteList(String limelightName) {
        vision = LimelightVision.getInstance();
        this.whitelistedArray = Field.ALL_TAGS;
        this.limelightName = limelightName;
    }

    @Override
    public void initialize() {
        vision.setTagWhitelist(whitelistedArray, limelightName);
    }
    
    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}