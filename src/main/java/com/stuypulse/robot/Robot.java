/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot;

import com.stuypulse.robot.commands.swerve.SwerveAutonInit;
import com.stuypulse.robot.commands.swerve.SwerveTeleopInit;
import com.stuypulse.robot.commands.vision.SetMegaTagMode;
import com.stuypulse.robot.commands.vision.WhitelistAllTagsForAllCameras;
import com.stuypulse.robot.commands.vision.WhitelistRoutineLeftSideAuto;
import com.stuypulse.robot.commands.vision.WhitelistRoutineRightSideAuto;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import com.stuypulse.robot.subsystems.vision.LimelightVision;
import com.stuypulse.robot.util.EnergyUtil;
import com.stuypulse.robot.util.PhoenixUtil;
import com.stuypulse.robot.util.superstructure.SOTMCalculator;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.lang.reflect.Field;
import java.util.List;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;

public class Robot extends TimedRobot {

    public enum RobotMode {
        DISABLED,
        AUTON,
        TELEOP,
        TEST
    }

    private RobotContainer robot;
    private Command auto;
    private static Alliance alliance;
    private static RobotMode mode;
    private static EnergyUtil energyUtil;
    private GcStatsCollector gcStatsCollector;

    private static int periodicCounter = 0;

    public static boolean isBlue() {
        return alliance == Alliance.Blue;
    }

    public static EnergyUtil getEnergyUtil() {
        return energyUtil;
    }

    public static RobotMode getMode() {
        return mode;
    }

    public static int getPeriodicCounter() {
        return periodicCounter;
    }

    /*************************/
    /*** ROBOT SCHEDULEING ***/
    /*************************/

    @Override
    public void robotInit() {
        robot = new RobotContainer();
        mode = RobotMode.DISABLED;
        energyUtil = new EnergyUtil();
        gcStatsCollector = new GcStatsCollector();

        try {
            Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
            watchdogField.setAccessible(true);
            Watchdog watchdog = (Watchdog) watchdogField.get(this);
            watchdog.setTimeout(Settings.LOOP_OVERRUN_WARNING_TIME_SEC);
        } catch (Exception e) {
            DriverStation.reportError("Failed to disable loop overrun warnings.", e.getStackTrace());
        }
        // this doesnt seem to work? 3/25 11:46AM

        DataLogManager.start();
        SignalLogger.start();
        CommandScheduler.getInstance().schedule(new SwerveAutonInit());
        FollowPathCommand.warmupCommand().schedule();
        PathfindingCommand.warmupCommand().schedule();
        energyUtil = new EnergyUtil();

        CommandScheduler.getInstance().schedule(new SwerveAutonInit());
    }

    @Override
    public void robotPeriodic() {
        PhoenixUtil.refreshAll();

        if (periodicCounter % 50 == 0) {
            DataLogManager.getLog().resume();
        }

        periodicCounter++;

        double batteryVoltage = RobotController.getBatteryVoltage();
        energyUtil.setBatteryVoltage(batteryVoltage);

        SuperstructureState state = Superstructure.getInstance().getState();
        if (state == SuperstructureState.SOTM) {
            SOTMCalculator.updateSOTMSolution();
        } else if (state == SuperstructureState.FOTM) {
            SOTMCalculator.updateFOTMSolution();
        }

        CommandScheduler.getInstance().run();

        robot.periodicAfterScheduler();

        if (!Robot.isReal()) {
            SmartDashboard.putData(CommandScheduler.getInstance());
        }

        if (DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.getAlliance().get();
        }

        SmartDashboard.putNumber("Robot/Match Time", DriverStation.getMatchTime());
        SmartDashboard.putData("Robot/Scheduled Commands", CommandScheduler.getInstance());
        SmartDashboard.putNumber("Robot/Battery Voltage", batteryVoltage);

        gcStatsCollector.update();
    }

    /*********************/
    /*** DISABLED MODE ***/
    /*********************/

    @Override
    public void disabledInit() {
        mode = RobotMode.DISABLED;

        CommandScheduler.getInstance().schedule(new SetMegaTagMode(LimelightVision.MegaTagMode.MEGATAG1));
    }

    @Override
    public void disabledPeriodic() {
        if (periodicCounter % Settings.LOGGING_FREQUENCY == 0) {
            auto = robot.getAutonomousCommand();
            switch (auto.getName()) {
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
        mode = RobotMode.AUTON;

        CommandScheduler.getInstance().schedule(new SetMegaTagMode(LimelightVision.MegaTagMode.MEGATAG2));
        CommandScheduler.getInstance().schedule(new WhitelistAllTagsForAllCameras());

        auto = robot.getAutonomousCommand();

        if (auto != null) {
            auto.schedule();
        }

    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        CommandScheduler.getInstance().schedule(new SwerveTeleopInit());
    }

    /*******************/
    /*** TELEOP MODE ***/
    /*******************/

    @Override
    public void teleopInit() {
        mode = RobotMode.TELEOP;

        CommandScheduler.getInstance().schedule(new SetMegaTagMode(LimelightVision.MegaTagMode.MEGATAG2));
        CommandScheduler.getInstance().schedule(new WhitelistAllTagsForAllCameras());

        if (auto != null) {
            auto.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    /*****************/
    /*** TEST MODE ***/
    /*****************/

    @Override
    public void testInit() {
        mode = RobotMode.TEST;
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    private static final class GcStatsCollector {
        private List<GarbageCollectorMXBean> gcBeans = ManagementFactory.getGarbageCollectorMXBeans();
        private final long[] lastTimes = new long[gcBeans.size()];
        private final long[] lastCounts = new long[gcBeans.size()];
        private int totalTime = 0;
        private int totalCount = 0;

        public void update() {
            long accumTime = 0;
            long accumCounts = 0;
            for (int i = 0; i < gcBeans.size(); i++) {
                long gcTime = gcBeans.get(i).getCollectionTime();
                long gcCount = gcBeans.get(i).getCollectionCount();
                accumTime += gcTime - lastTimes[i];
                accumCounts += gcCount - lastCounts[i];

                lastTimes[i] = gcTime;
                lastCounts[i] = gcCount;
            }

            totalTime += (int) accumTime;
            totalCount += (int) accumCounts;

            SmartDashboard.putNumber("Robot/GC Time MS", (double) accumTime);
            SmartDashboard.putNumber("Robot/GC Counts", (double) accumCounts);
            SmartDashboard.putNumber("Robot/Sum of GC Time MS", totalTime);
            SmartDashboard.putNumber("Robot/Sum of GC Counts", totalCount);

            // Logger.recordOutput("LoggedRobot/GCTimeMS", (double) accumTime);
            // Logger.recordOutput("LoggedRobot/GCCounts", (double) accumCounts);
        }
    }

}
