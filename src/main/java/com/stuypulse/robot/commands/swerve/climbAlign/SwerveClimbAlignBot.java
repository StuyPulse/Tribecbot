/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.swerve.climbAlign;

import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.pidToPose.SwerveDrivePIDToPose;
import com.stuypulse.robot.constants.DriverConstants;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.DriverConstants.Driver.Turn;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class SwerveClimbAlignBot extends SwerveDrivePIDToPose{
    private final Gamepad driver;
    public SwerveClimbAlignBot(Gamepad driver){
        super(new Pose2d(Field.towerFarCenter.getX(), Field.towerFarCenter.getY() - Field.barDisplacement - Field.DISTANCE_TO_RUNGS, new Rotation2d(0)));
        this.driver = driver;
    }

    @Override
    public void execute(){
        super.execute();
    }

    @Override
    public boolean isFinished() {
        if (driver != null) {
            boolean isFinished = (driver.getLeftStick().distance() > DriverConstants.Driver.Drive.DEADBAND
                    || driver.getRightStick().distance() > DriverConstants.Driver.Turn.DEADBAND) ? true : false;
            SmartDashboard.putBoolean("Driver/Stick Moved", isFinished);
            return isFinished;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        CommandScheduler.getInstance().schedule(new SwerveDriveDrive(driver));
    }
    
    
}