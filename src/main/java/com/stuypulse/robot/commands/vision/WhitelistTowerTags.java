package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;

public class WhitelistTowerTags extends SetLimelightWhiteList {
    public WhitelistTowerTags (String limelightName) {
        super(Robot.isBlue() ? Field.BLUE_TOWER_TAG_IDS: Field.RED_TOWER_TAG_IDS, limelightName);
    }
}