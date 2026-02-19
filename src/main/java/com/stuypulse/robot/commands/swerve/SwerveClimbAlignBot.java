/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.commands.swerve.pidToPose.SwerveDrivePIDToPose;
import com.stuypulse.robot.constants.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveClimbAlignBot extends SwerveDrivePIDToPose{
    public SwerveClimbAlignBot(){
        super(new Pose2d(Field.towerCenter.getX(), Field.towerCenter.getY() - Field.barDisplacement - Field.DISTANCE_TO_RUNGS, new Rotation2d(0)));
    }

    @Override
    public void execute(){
        super.execute();
    }
}