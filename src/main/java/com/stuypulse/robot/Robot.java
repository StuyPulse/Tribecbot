/** ********************** PROJECT TRIBECBOT ************************ */
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
 /* Use of this source code is governed by an MIT-style license */
 /* that can be found in the repository LICENSE file.           */
/** ************************************************************ */
package com.stuypulse.robot;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.lang.reflect.Field;
import java.util.List;
import java.util.Timer;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.stuypulse.robot.commands.handoff.HandoffStop;
import com.stuypulse.robot.commands.spindexer.SpindexerStop;
import com.stuypulse.robot.commands.superstructure.SuperstructureFOTM;
import com.stuypulse.robot.commands.superstructure.SuperstructureSOTM;
import com.stuypulse.robot.commands.swerve.SwerveAutonInit;
import com.stuypulse.robot.commands.swerve.SwerveTeleopInit;
import com.stuypulse.robot.commands.vision.SetMegaTagMode;
import com.stuypulse.robot.commands.vision.WhitelistAllTagsForAllCameras;
import com.stuypulse.robot.commands.vision.WhitelistRoutineLeftSideAuto;
import com.stuypulse.robot.commands.vision.WhitelistRoutineRightSideAuto;
import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Cameras.Camera;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.handoff.HandoffImpl;
import com.stuypulse.robot.subsystems.spindexer.Spindexer;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.vision.LimelightVision;
import com.stuypulse.robot.util.EnergyUtil;
import com.stuypulse.robot.util.MasterLogger;
import com.stuypulse.robot.util.MotorLogger.SubsystemName;
import com.stuypulse.robot.util.FMSUtil;
import com.stuypulse.robot.util.PhoenixUtil;
import com.stuypulse.robot.util.superstructure.SOTMCalculator;
import com.stuypulse.stuylib.network.SmartBoolean;

import dev.doglog.DogLogOptions;
import dev.doglog.DogLog;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.util.List;
import java.util.Timer;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.stuypulse.robot.commands.handoff.HandoffStop;
import com.stuypulse.robot.commands.spindexer.SpindexerStop;
import com.stuypulse.robot.commands.superstructure.SuperstructureFOTM;
import com.stuypulse.robot.commands.vision.BlackListAllTagsForAllCameras;
import com.stuypulse.robot.commands.vision.BlacklistAllTags;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
public class Robot extends TimedRobot {

    public enum RobotMode {
        DISABLED,
        AUTON,
        TELEOP,
        TEST
    }

    private Timer threadTimer;

    private RobotContainer robot;
    private Command auto;
    private static Alliance alliance;
    private static RobotMode mode;
    private static EnergyUtil energyUtil;
    private FMSUtil fmsUtil;
    private SendableChooser<Camera> cameras = new SendableChooser<Camera>();
    private Camera selected;
    private GcStatsCollector gcStatsCollector;
    private SmartBoolean shouldRunSecondThread;

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

    /**
     * **********************
     */
    /**
     * * ROBOT SCHEDULEING **
     */
    /**
     * **********************
     */
    @Override
    public void robotInit() {
        robot = new RobotContainer();
        mode = RobotMode.DISABLED;
        energyUtil = new EnergyUtil();
        fmsUtil = new FMSUtil(true);
        gcStatsCollector = new GcStatsCollector();

        DataLogManager.start();
        SignalLogger.start();
        CommandScheduler.getInstance().schedule(new SwerveAutonInit());
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
        energyUtil = new EnergyUtil();

        CommandScheduler.getInstance().schedule(new SwerveAutonInit());
        
        DogLog.setOptions(new DogLogOptions().withNtPublish(false)); //should only be in logs
    }
    @Override
    public void robotPeriodic() {
        PhoenixUtil.refreshAll();
        CommandScheduler.getInstance().run();

        if (periodicCounter % 50 == 0) {
            DataLogManager.getLog().resume();
        }
        if (cameras.getSelected() != selected) {
            PortForwarder.remove(5801);
            selected = cameras.getSelected();
            PortForwarder.add(5801, selected + ".local:5801", 5801);
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

        if (!Robot.isReal()) {
            SmartDashboard.putData(CommandScheduler.getInstance());
        }

        if (DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.getAlliance().get();
        }

        if (CommandSwerveDrivetrain.getInstance().isOutsideAllianceZone() && Superstructure.getInstance().getState() == SuperstructureState.SOTM) {
            CommandScheduler.getInstance().schedule(
                    new SuperstructureFOTM(),
                    new SpindexerStop(),
                    new HandoffStop()
            );
        }

        SmartDashboard.putNumber("Robot/Match Time", DriverStation.getMatchTime());
        SmartDashboard.putData("Robot/Scheduled Commands", CommandScheduler.getInstance());
        SmartDashboard.putNumber("Robot/Battery Voltage", batteryVoltage);
        SmartDashboard.putNumber("Robot/CPU Temperature (C)", RobotController.getCPUTemp());

        robot.periodicAfterScheduler();

        //DO NOT wrap this inside of if(Settings.DebugMode) -> this is now being accounted for inside of EnergyUtil
        //theoretically we would need to overlay both the SmartDashboard supplyCurrent graph with the DogLog supplyCurrent graph to see the full picture
        //(was thinking instead of this we should make a seperate method inside of energyUtil that runs all the time with DogLog stuff and keep this "periodic" method inside of DebugMode seperately)
        //such so we always log to doglog but can always turn debug mode off and visualize, and then not have to deal with overlapping the graphs.
        energyUtil.periodic();
        //read above comments
    }

    /**
     * ******************
     */
    /**
     * * DISABLED MODE **
     */
    /**
     * ******************
     */
    @Override
    public void disabledInit() {
        mode = RobotMode.DISABLED;

        CommandScheduler.getInstance().schedule(new SetMegaTagMode(LimelightVision.MegaTagMode.MEGATAG2));

        CommandScheduler.getInstance().schedule(new BlackListAllTagsForAllCameras());

    }

    @Override
    public void disabledPeriodic() {
        if (periodicCounter % Settings.LOGGING_FREQUENCY == 0) {
            auto = robot.getAutonomousCommand();
        }
    }

    /**
     * ********************
     */
    /**
     * * AUTONOMOUS MODE **
     */
    /**
     * ********************
     */
    @Override
    public void autonomousInit() {
        mode = RobotMode.AUTON;
        CommandScheduler.getInstance().schedule(new SetMegaTagMode(LimelightVision.MegaTagMode.MEGATAG2));
        CommandScheduler.getInstance().schedule(new WhitelistAllTagsForAllCameras());

        auto = robot.getAutonomousCommand();

        if (auto != null) {
            CommandScheduler.getInstance().schedule(auto);
        }

    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        CommandScheduler.getInstance().schedule(new SwerveTeleopInit());
    }

    /**
     * ****************
     */
    /**
     * * TELEOP MODE **
     */
    /**
     * ****************
     */
    @Override
    public void teleopInit() {
        mode = RobotMode.TELEOP;
        fmsUtil.restartTimer(false);
        CommandScheduler.getInstance().schedule(new SetMegaTagMode(LimelightVision.MegaTagMode.MEGATAG2));
        CommandScheduler.getInstance().schedule(new WhitelistAllTagsForAllCameras());

        if (auto != null) {
            auto.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
        SmartDashboard.putNumber("FMSUtil/time left in shift", fmsUtil.getTimeLeftInShift());
        SmartDashboard.putBoolean("FMSUtil/is active shift", fmsUtil.isActiveShift());
        SmartDashboard.putBoolean("FMSUtil/won auto?", fmsUtil.didWinAuto());
    }

    @Override
    public void teleopExit() {
    }

    /**
     * **************
     */
    /**
     * * TEST MODE **
     */
    /**
     * **************
     */
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
