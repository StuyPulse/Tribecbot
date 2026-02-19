/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.util.vision;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Field.NamedTags;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public interface HubUtil {

    public final Pose2d blueHubCenter = new Pose2d(Units.inchesToMeters(158.60), Units.inchesToMeters(Field.WIDTH / 2.0), new Rotation2d());

    public static Pose2d getAllianceHubPose() {
        return (Robot.isBlue() ? blueHubCenter : Field.transformToOppositeAlliance(blueHubCenter));
    }

    public enum FerryTargetPositions {

        RIGHT_WALL(new Pose2d(1.0, 1.0, new Rotation2d())),
        LEFT_WALL(new Pose2d());

        private Pose2d targetPosition;

        private FerryTargetPositions(Pose2d targetPosition) {
            this.targetPosition = targetPosition;
        }

        public Pose2d getFerryTargetPose() {
            return Robot.isBlue() ? targetPosition : Field.transformToOppositeAlliance(targetPosition);
        }
    }

    public enum HubTag {
        FRONT_MID(NamedTags.BLUE_HUB_FRONT_SIDE_MID, NamedTags.RED_HUB_FRONT_SIDE_MID),
        FRONT_LEFT(NamedTags.BLUE_HUB_FRONT_SIDE_LEFT, NamedTags.RED_HUB_FRONT_SIDE_LEFT),
        LEFT_RIGHT(NamedTags.BLUE_HUB_LEFT_SIDE_RIGHT, NamedTags.RED_HUB_LEFT_SIDE_RIGHT),
        LEFT_MID(NamedTags.BLUE_HUB_LEFT_SIDE_MID, NamedTags.RED_HUB_LEFT_SIDE_MID),
        BACK_MID(NamedTags.BLUE_HUB_BACK_SIDE_MID, NamedTags.RED_HUB_BACK_SIDE_MID),
        BACK_LEFT(NamedTags.BLUE_HUB_BACK_SIDE_LEFT, NamedTags.RED_HUB_BACK_SIDE_LEFT),
        RIGHT_MID(NamedTags.BLUE_HUB_RIGHT_SIDE_MID, NamedTags.RED_HUB_RIGHT_SIDE_MID),
        RIGHT_LEFT(NamedTags.BLUE_HUB_RIGHT_SIDE_LEFT, NamedTags.RED_HUB_RIGHT_SIDE_LEFT);

        private NamedTags correspondingBlueAprilTag;
        private NamedTags correspondingRedAprilTag;

        private HubTag(NamedTags correspondingBlueAprilTag, NamedTags correspondingRedAprilTag) {
            this.correspondingBlueAprilTag = correspondingBlueAprilTag;
            this.correspondingRedAprilTag = correspondingRedAprilTag;
        }

        public Pose2d getCorrespondingAprilTagPose() {
            return Robot.isBlue() ? this.correspondingBlueAprilTag.getLocation().toPose2d() : this.correspondingRedAprilTag.getLocation().toPose2d();
        }
    }

    public enum HubFace {
        FRONT(HubTag.FRONT_MID, HubTag.FRONT_LEFT),
        LEFT(HubTag.LEFT_MID, HubTag.LEFT_RIGHT),
        BACK(HubTag.BACK_MID, HubTag.BACK_LEFT),
        RIGHT(HubTag.RIGHT_MID, HubTag.RIGHT_LEFT);

        private HubTag midTag;
        private HubTag nonMidTag;

        private HubFace(HubTag midTag, HubTag nonMidTag) {
            this.midTag = midTag;
            this.nonMidTag = nonMidTag;
        }

        public HubTag getMidHubTag() {
            return this.midTag;
        }

        public HubTag getNonMidHubTag() {
            return this.nonMidTag;
        }
    }
}