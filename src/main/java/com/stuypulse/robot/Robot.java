/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot;

import com.stuypulse.robot.commands.swerve.SwerveAutonInit;
import com.stuypulse.robot.commands.vision.SetMegaTagMode;
import com.stuypulse.robot.commands.vision.WhitelistAllTags;
import com.stuypulse.robot.commands.vision.WhitelistAllTagsForAllCameras;
import com.stuypulse.robot.commands.vision.WhitelistRoutineLeftSideAuto;
import com.stuypulse.robot.commands.vision.WhitelistRoutineRightSideAuto;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.vision.LimelightVision;
import com.stuypulse.robot.util.EnergyUtil;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.ctre.phoenix6.SignalLogger;

public class Robot extends TimedRobot {

    private RobotContainer robot;
    private static Alliance alliance;
    private static int periodicCounter = 0;
    private Command selectedAuto;
    private static EnergyUtil energyUtil;
    private static final Timer timer = new Timer();

    public static boolean isBlue() {
        return alliance == Alliance.Blue;
    }

    public static EnergyUtil getEnergyUtil() {
        return energyUtil;
    }

    public static double getRobotTime() {
        return timer.get();
    }

    /*************************/
    /*** ROBOT SCHEDULEING ***/
    /*************************/

    @Override
    public void robotInit() {
        robot = new RobotContainer();
        selectedAuto = robot.getAutonomousCommand();
        timer.start();

        DataLogManager.start();
        SignalLogger.start();
        energyUtil = new EnergyUtil();
    }
    
    public static int getPeriodicCounter() {
        return periodicCounter;
    }

    @Override
    public void robotPeriodic() {
        if (periodicCounter % 50 == 0) {
            DataLogManager.getLog().resume();
        }

        periodicCounter++;

        double batteryVoltage = RobotController.getBatteryVoltage();
        energyUtil.setBatteryVoltage(batteryVoltage);
        robot.logEnergyForAllSubsystems(energyUtil);
        CommandScheduler.getInstance().run();
        if (!Robot.isReal()) {
            SmartDashboard.putData(CommandScheduler.getInstance());
        }

        SmartDashboard.putNumber("Robot/Match Time", DriverStation.getMatchTime());
        SmartDashboard.putData("Robot/Scheduled Commands", CommandScheduler.getInstance());
        SmartDashboard.putNumber("Robot/Battery Voltage", batteryVoltage);
        
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
        
        mode = RobotMode.DISABLED;
    }

    @Override
    public void disabledPeriodic() {
        if (periodicCounter % Settings.LOGGING_FREQUENCY == 0) {
            selectedAuto = robot.getAutonomousCommand();
            switch (selectedAuto.getName()) {
                case "LeftTwoCycle":
                    CommandScheduler.getInstance().schedule(new WhitelistRoutineLeftSideAuto());
                    break;
                case "RightTwoCycle":
                    CommandScheduler.getInstance().schedule(new WhitelistRoutineRightSideAuto());
                    break;
                default:
                    CommandScheduler.getInstance().schedule(new WhitelistAllTagsForAllCameras());
                    break;
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
        CommandScheduler.getInstance().schedule(new WhitelistAllTagsForAllCameras());

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
        CommandScheduler.getInstance().schedule(new WhitelistAllTagsForAllCameras());

        if (auto != null) {
            auto.cancel();
        }
    }

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
