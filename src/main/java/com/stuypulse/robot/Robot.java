/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot;

import com.stuypulse.robot.commands.vision.SetIMUMode;
import com.stuypulse.robot.commands.vision.SetMegaTagMode;
import com.stuypulse.robot.constants.Settings;
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

    public static boolean isBlue() {
        return alliance == Alliance.Blue;
    }

    /*************************/
    /*** ROBOT SCHEDULEING ***/
    /*************************/

    @Override
    public void robotInit() {
        robot = new RobotContainer();

        DataLogManager.start();
        SignalLogger.start();
        CommandScheduler.getInstance().schedule(new SetIMUMode(Settings.Vision.RESET_IMU_INDEX));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        if (!Robot.isReal()) {
            SmartDashboard.putData(CommandScheduler.getInstance());
        }

        SmartDashboard.putNumber("Robot/Match Time", DriverStation.getMatchTime());

        if (DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.getAlliance().get();
        }
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
        CommandScheduler.getInstance().schedule(new SetIMUMode(Settings.Vision.RESET_IMU_INDEX));
    }

    /***********************/
    /*** AUTONOMOUS MODE ***/
    /***********************/  

    @Override
    public void autonomousInit() {

        CommandScheduler.getInstance().schedule(new SetMegaTagMode(LimelightVision.MegaTagMode.MEGATAG2));
        CommandScheduler.getInstance().schedule(new SetIMUMode(Settings.Vision.INTERNAL_EXTERNAL_ASSIST_INDEX));

        auto = robot.getAutonomousCommand();

        if (auto != null) {
            auto.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit(
        
    ) {}

    /*******************/
    /*** TELEOP MODE ***/
    /*******************/

    @Override
    public void teleopInit() {

        CommandScheduler.getInstance().schedule(new SetMegaTagMode(LimelightVision.MegaTagMode.MEGATAG2));
        CommandScheduler.getInstance().schedule(new SetIMUMode(0));

        if (auto != null) {
            auto.cancel();
        }

    }

    @Override
    public void teleopPeriodic() {
    }

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
