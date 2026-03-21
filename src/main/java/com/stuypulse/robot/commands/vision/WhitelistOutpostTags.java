package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;

public class WhitelistOutpostTags extends SetLimelightWhiteList {
    public WhitelistOutpostTags (String limelightName) {
        super(Robot.isBlue() ? Field.BLUE_OUTPOST_TAG_IDS: Field.RED_OUTPOST_TAG_IDS, limelightName);
    }
}