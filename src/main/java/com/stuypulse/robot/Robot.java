/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot;

import com.stuypulse.robot.commands.swerve.SwerveAutonInit;
import com.stuypulse.robot.commands.vision.SetMegaTagMode;
import com.stuypulse.robot.commands.vision.WhitelistAllTags;
import com.stuypulse.robot.commands.vision.WhitelistRoutineLeftSideAuto;
import com.stuypulse.robot.commands.vision.WhitelistRoutineRightSideAuto;
import com.stuypulse.robot.subsystems.vision.LimelightVision;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.ctre.phoenix6.SignalLogger;

public class Robot extends TimedRobot {

    private RobotContainer robot;
    private Command auto;
    private static Alliance alliance;
    private int resetLoggingCounter = 0;
    private Command selectedAuto;

    public static boolean isBlue() {
        return alliance == Alliance.Blue;
    }

    /*************************/
    /*** ROBOT SCHEDULEING ***/
    /*************************/

    @Override
    public void robotInit() {
        robot = new RobotContainer();
        selectedAuto = robot.getAutonomousCommand();

        DataLogManager.start();
        SignalLogger.start();
    }

    @Override
    public void robotPeriodic() {
        if (resetLoggingCounter % 50 == 0) {
            DataLogManager.getLog().resume();
        }

        resetLoggingCounter++;

        
        CommandScheduler.getInstance().run();
        if (!Robot.isReal()) {
            SmartDashboard.putData(CommandScheduler.getInstance());
        }

        SmartDashboard.putNumber("Robot/Match Time", DriverStation.getMatchTime());
        SmartDashboard.putData("Robot/Scheduled Commands", CommandScheduler.getInstance());
        
        if (DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.getAlliance().get();
        }

        robot.periodic();
    }

    /*********************/
    /*** DISABLED MODE ***/
    /*********************/

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().schedule(new SetMegaTagMode(LimelightVision.MegaTagMode.MEGATAG1));
    }

    @Override
    public void disabledPeriodic() {
        if (resetLoggingCounter % 50 == 0) {
            selectedAuto = robot.getAutonomousCommand();

            switch (selectedAuto.getName()) {
                case "Left Two Cycle":
                    CommandScheduler.getInstance().schedule(new WhitelistRoutineLeftSideAuto());
                    System.out.println("LEFT SCHEDULED");
                case "Right Two Cycle":
                    CommandScheduler.getInstance().schedule(new WhitelistRoutineRightSideAuto());
                    System.out.println("RIGHT SCHEDULED");
        }
        }
    }

    /***********************/
    /*** AUTONOMOUS MODE ***/
    /***********************/  

    @Override 
    public void autonomousInit() {
        CommandScheduler.getInstance().schedule(new SetMegaTagMode(LimelightVision.MegaTagMode.MEGATAG2));
        CommandScheduler.getInstance().schedule(new SwerveAutonInit());
        CommandScheduler.getInstance().schedule(new WhitelistAllTags("limelight-left"));
        CommandScheduler.getInstance().schedule(new WhitelistAllTags("limelight-right"));
        CommandScheduler.getInstance().schedule(new WhitelistAllTags("limelight-back"));

        auto = robot.getAutonomousCommand();

        if (auto != null) {
            auto.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    /*******************/
    /*** TELEOP MODE ***/
    /*******************/

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().schedule(new SetMegaTagMode(LimelightVision.MegaTagMode.MEGATAG2));

        
        if (auto != null) {
            auto.cancel();
        }

    }
// 
    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    /*****************/
    /*** TEST MODE ***/
    /*****************/

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
