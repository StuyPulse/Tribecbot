package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;

public class WhitelistTrenchTags extends SetLimelightWhiteList {
    public WhitelistTrenchTags(String limelightName) {
        super(Robot.isBlue() ? Field.BLUE_TRENCH_TAG_IDS : Field.RED_TRENCH_TAG_IDS, limelightName);
    }
}
