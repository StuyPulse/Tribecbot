/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;
import com.stuypulse.stuylib.network.SmartBoolean;

import com.stuypulse.robot.commands.BuzzController;
import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.climberhopper.ClimberHopperDefaultCommand;
import com.stuypulse.robot.commands.handoff.HandoffReverse;
import com.stuypulse.robot.commands.handoff.HandoffRun;
import com.stuypulse.robot.commands.handoff.HandoffStop;
import com.stuypulse.robot.commands.hoodedshooter.HoodedShooterFerry;
import com.stuypulse.robot.commands.hoodedshooter.HoodedShooterInterpolation;
import com.stuypulse.robot.commands.hoodedshooter.HoodedShooterLeftCorner;
import com.stuypulse.robot.commands.hoodedshooter.HoodedShooterReverse;
import com.stuypulse.robot.commands.hoodedshooter.HoodedShooterRightCorner;
import com.stuypulse.robot.commands.hoodedshooter.HoodedShooterShoot;
import com.stuypulse.robot.commands.hoodedshooter.HoodedShooterStow;
import com.stuypulse.robot.commands.intake.IntakeIntake;
import com.stuypulse.robot.commands.intake.IntakeOutake;
import com.stuypulse.robot.commands.intake.IntakeStop;
import com.stuypulse.robot.commands.leds.LEDApplyState;
import com.stuypulse.robot.commands.leds.LEDDefaultCommand;
import com.stuypulse.robot.commands.spindexer.SpindexerRun;
import com.stuypulse.robot.commands.spindexer.SpindexerStop;
import com.stuypulse.robot.commands.swerve.SwerveClimbAlign;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveResetHeading;
import com.stuypulse.robot.commands.swerve.SwerveXMode;
import com.stuypulse.robot.commands.turret.TurretFerry;
import com.stuypulse.robot.commands.turret.TurretIdle;
import com.stuypulse.robot.commands.turret.TurretLeftCorner;
import com.stuypulse.robot.commands.turret.TurretRightCorner;
import com.stuypulse.robot.commands.turret.TurretSeed;
import com.stuypulse.robot.commands.turret.TurretShoot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.climberhopper.ClimberHopper;
import com.stuypulse.robot.subsystems.handoff.Handoff;
import com.stuypulse.robot.subsystems.hoodedshooter.HoodedShooter;
import com.stuypulse.robot.subsystems.hoodedshooter.hood.Hood;
import com.stuypulse.robot.subsystems.hoodedshooter.shooter.Shooter;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.leds.LEDController;
import com.stuypulse.robot.subsystems.spindexer.Spindexer;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.turret.Turret;
import com.stuypulse.robot.subsystems.vision.LimelightVision;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;


public class RobotContainer {
    public interface EnabledSubsystems {
        SmartBoolean SWERVE = new SmartBoolean("Enabled Subsystems/Swerve Is Enabled", true);
        SmartBoolean TURRET = new SmartBoolean("Enabled Subsystems/Turret Is Enabled", false);
        SmartBoolean HANDOFF = new SmartBoolean("Enabled Subsystems/Handoff Is Enabled", false);
        SmartBoolean INTAKE = new SmartBoolean("Enabled Subsystems/Intake Is Enabled", false);
        SmartBoolean SPINDEXER = new SmartBoolean("Enabled Subsystems/Spindexer Is Enabled", false);
        SmartBoolean CLIMBER_HOPPER = new SmartBoolean("Enabled Subsystems/Climber-Hopper Is Enabled", false);
        SmartBoolean HOOD = new SmartBoolean("Enabled Subsystems/Hood Is Enabled", false);
        SmartBoolean SHOOTER = new SmartBoolean("Enabled Subsystems/Shooter Is Enabled", false);
        SmartBoolean LIMELIGHT = new SmartBoolean("Enabled Subsystems/Limelight Is Enabled", false);
        SmartBoolean LEDS = new SmartBoolean("Enabled Subsystems/LEDs Is Enabled", false);
    }

    // Gamepads
    public static final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public static final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);


    // Subsystem
    private final ClimberHopper climberHopper = ClimberHopper.getInstance();
    private final Handoff handoff = Handoff.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Spindexer spindexer = Spindexer.getInstance();
    
    private final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
    private final LimelightVision vision = LimelightVision.getInstance();
    private final Turret turret = Turret.getInstance();

    private final HoodedShooter hoodedShooter = HoodedShooter.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Hood hood = Hood.getInstance();
    private final LEDController leds = LEDController.getInstance();

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot container
    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();

        SmartDashboard.putData("Field", Field.FIELD2D);
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new SwerveDriveDrive(driver));
        climberHopper.setDefaultCommand(new ClimberHopperDefaultCommand());
        leds.setDefaultCommand(new LEDDefaultCommand());
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {

        driver.getDPadDown()
            .onTrue(new TurretIdle())
            .onTrue(new TurretSeed());

        driver.getDPadUp()
            .onTrue(new SwerveResetHeading());
        
        // SCORING ROUTINE
        driver.getTopButton()
                .whileTrue(new TurretShoot()
                        .alongWith(new HoodedShooterShoot())
                        .alongWith(new WaitUntilCommand(() -> hoodedShooter.bothAtTolerance()))
                        .andThen(new HandoffRun().onlyIf(() -> hoodedShooter.bothAtTolerance())
                                .alongWith(new WaitUntilCommand(() -> handoff.atTolerance()))
                                .andThen(new SpindexerRun().onlyIf(() -> handoff.atTolerance() && hoodedShooter.bothAtTolerance()))))
                .onFalse(new SpindexerStop()
                        .alongWith(new HoodedShooterStow())
                        .alongWith(new HandoffStop()));

        driver.getTopButton()
                .whileTrue(new TurretShoot()
                        .alongWith(new HoodedShooterInterpolation())
                        .alongWith(new WaitUntilCommand(() -> hoodedShooter.bothAtTolerance()))
                        .andThen(new HandoffRun().onlyIf(() -> hoodedShooter.bothAtTolerance())
                                .alongWith(new WaitUntilCommand(() -> handoff.atTolerance()))
                                .andThen(new SpindexerRun().onlyIf(() -> handoff.atTolerance() && hoodedShooter.bothAtTolerance()))))
                .onFalse(new SpindexerStop()
                        .alongWith(new HoodedShooterStow())
                        .alongWith(new HandoffStop()));

        // driver.getDPadDown()
        //     .onTrue(new HoodedShooterShoot())
        //     .onFalse(new HoodedShooterStow());

        // driver.getDPadUp()
        //     .onTrue(new HoodedShooterFerry())
        //     .onFalse(new HoodedShooterStow());

        // driver.getDPadUp().whileTrue(new HoodedShooterShoot()
        //     .alongWith(new WaitUntilCommand(() -> hoodedShooter.isShooterAtTolerance())
        //     .andThen(new FeederFeed())))
        // .onFalse(new HoodedShooterStow()
        //     .alongWith(new FeederStop()));

//-------------------------------------------------------------------------------------------------------------------------\\
//-------------------------------------------------------------------------------------------------------------------------\\
//-------------------------------------------------------------------------------------------------------------------------\\
//-------------------------------------------------------------------------------------------------------------------------\\
//-------------------------------------------------------------------------------------------------------------------------\\
        // Climb Align
        driver.getTopButton()
            .whileTrue(
                new SwerveClimbAlign().alongWith(
                    new LEDApplyState(Settings.LEDS.LEDState.PRESSED_TOP_BUTTON))
                );

        // Left Corner Shoot
        driver.getLeftButton()
            .whileTrue(
                new SwerveXMode().alongWith(
                    new HoodedShooterLeftCorner().alongWith(
                        new LEDApplyState(Settings.LEDS.LEDState.PRESSED_LEFT_BUTTON).alongWith(
                            new TurretLeftCorner())).alongWith(
                                new WaitUntilCommand(() -> hoodedShooter.isHoodAtTolerance())).alongWith(
                                new WaitUntilCommand(() -> hoodedShooter.isShooterAtTolerance())).alongWith(
                                new WaitUntilCommand(() -> turret.atTargetAngle())).andThen(
                                    new SpindexerRun().alongWith(new HandoffRun()))))
            .onFalse(
                new HoodedShooterStow().alongWith(
                new LEDApplyState(Settings.LEDS.LEDState.DEFAULT_SETTING).alongWith(
                new SpindexerRun()).alongWith(
                new HandoffStop()))
            );

        // Right Corner Shoot
        driver.getRightButton()
            .whileTrue(
                new SwerveXMode().alongWith(
                    new HoodedShooterRightCorner().alongWith(
                        new LEDApplyState(Settings.LEDS.LEDState.PRESSED_RIGHT_BUTTON).alongWith(
                            new TurretRightCorner())).alongWith(
                                new WaitUntilCommand(() -> hoodedShooter.isHoodAtTolerance())).alongWith(
                                new WaitUntilCommand(() -> hoodedShooter.isShooterAtTolerance())).alongWith(
                                new WaitUntilCommand(() -> turret.atTargetAngle())).andThen(
                                    new SpindexerRun().alongWith(new HandoffRun()))))
            .onFalse(
                new HoodedShooterStow().alongWith(
                new LEDApplyState(Settings.LEDS.LEDState.DEFAULT_SETTING).alongWith(
                new SpindexerRun()).alongWith(
                new HandoffStop()))
            );

        // Hub Shoot
        driver.getBottomButton()
            .whileTrue(
                new SwerveXMode().alongWith(
                    new HoodedShooterShoot().alongWith(
                        new LEDApplyState(Settings.LEDS.LEDState.PRESSED_BOT_BUTTON).alongWith(
                            new TurretShoot())).alongWith(
                                new WaitUntilCommand(() -> hoodedShooter.isHoodAtTolerance())).alongWith(
                                new WaitUntilCommand(() -> hoodedShooter.isShooterAtTolerance())).alongWith(
                                new WaitUntilCommand(() -> turret.atTargetAngle())).andThen(
                                    new SpindexerRun().alongWith(new HandoffRun()))))
            .onFalse(
                new HoodedShooterStow().alongWith(
                new LEDApplyState(Settings.LEDS.LEDState.DEFAULT_SETTING).alongWith(
                new SpindexerRun()).alongWith(
                new HandoffStop()))
            );

        // Intake On
        driver.getLeftTriggerButton()
            .onTrue(
                new IntakeIntake().alongWith(
                    new LEDApplyState(Settings.LEDS.LEDState.PRESSED_LEFT_TRIGGER)))
            .onFalse(
                new LEDApplyState(Settings.LEDS.LEDState.DEFAULT_SETTING)
            );

        // Intake Off
        driver.getRightTriggerButton()
            .onTrue(new IntakeStop().alongWith(
                new LEDApplyState(Settings.LEDS.LEDState.PRESSED_RIGHT_TRIGGER)
            ))
            .onFalse(
                new LEDApplyState(Settings.LEDS.LEDState.DEFAULT_SETTING)
            );

        // Climb Down Placeholder
        driver.getLeftBumper()
            .onTrue(new BuzzController(driver).alongWith(
                new LEDApplyState(Settings.LEDS.LEDState.PRESSED_LEFT_BUMPER)
            ))
            .onFalse(
                new LEDApplyState(Settings.LEDS.LEDState.DEFAULT_SETTING)
            );

        // Climb Up Placeholder
        driver.getRightBumper()
            .onTrue(new BuzzController(driver).alongWith(
                new LEDApplyState(Settings.LEDS.LEDState.PRESSED_RIGHT_BUMPER)
            ))
            .onFalse(
                new LEDApplyState(Settings.LEDS.LEDState.DEFAULT_SETTING)
            );

        // Reset Heading
        driver.getDPadUp()
            .onTrue(new SwerveResetHeading().alongWith(
                new LEDApplyState(Settings.LEDS.LEDState.PRESSED_TOP_DPAD)
            ))
            .onFalse(
                new LEDApplyState(Settings.LEDS.LEDState.DEFAULT_SETTING)
            );

        // Ferry In Place
        driver.getDPadLeft()
            .whileTrue(
                new SwerveXMode().alongWith(
                    new HoodedShooterFerry().alongWith(
                        new LEDApplyState(Settings.LEDS.LEDState.PRESSED_LEFT_DPAD).alongWith(
                            new TurretFerry())).alongWith(
                                new WaitUntilCommand(() -> hoodedShooter.isHoodAtTolerance())).alongWith(
                                new WaitUntilCommand(() -> hoodedShooter.isShooterAtTolerance())).alongWith(
                                new WaitUntilCommand(() -> turret.atTargetAngle())).andThen(
                                    new SpindexerRun().alongWith(new HandoffRun()))))
            .onFalse(
                new HoodedShooterStow().alongWith(
                new LEDApplyState(Settings.LEDS.LEDState.DEFAULT_SETTING)).alongWith(
                new SpindexerRun()).alongWith(
                new HandoffStop()));

        // Score In Place
        driver.getDPadRight()
            .whileTrue(
                new SwerveXMode().alongWith(
                    new HoodedShooterShoot().alongWith(
                        new LEDApplyState(Settings.LEDS.LEDState.PRESSED_RIGHT_DPAD).alongWith(
                        new TurretShoot()).alongWith(
                            new WaitUntilCommand(() -> hoodedShooter.isHoodAtTolerance())).alongWith(
                            new WaitUntilCommand(() -> hoodedShooter.isShooterAtTolerance())).alongWith(
                            new WaitUntilCommand(() -> turret.atTargetAngle())).andThen(
                                new SpindexerRun().alongWith(new HandoffRun())))))
            .onFalse(
                new HoodedShooterStow().alongWith(
                new LEDApplyState(Settings.LEDS.LEDState.DEFAULT_SETTING)).alongWith(
                new SpindexerRun()).alongWith(
                new HandoffStop()));
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public void configureSysids() {

        // autonChooser.addOption("SysID Module Translation Dynamic Forward", swerve.sysIdDynamic(Direction.kForward));
        // autonChooser.addOption("SysID Module Translation Dynamic Backwards", swerve.sysIdDynamic(Direction.kReverse));
        // autonChooser.addOption("SysID Module Translation Quasi Forwards", swerve.sysIdQuasistatic(Direction.kForward));
        // autonChooser.addOption("SysID Module Translation Quasi Backwards", swerve.sysIdQuasistatic(Direction.kReverse));

        // autonChooser.addOption("SysID Module Rotation Dynamic Forwards", swerve.sysIdRotDynamic(Direction.kForward));
        // autonChooser.addOption("SysID Module Rotation Dynamic Backwards", swerve.sysIdRotDynamic(Direction.kReverse));
        // autonChooser.addOption("SysID Module Rotation Quasi Forwards", swerve.sysIdRotQuasi(Direction.kForward));
        // autonChooser.addOption("SysID Module Rotation Quasi Backwards", swerve.sysIdRotQuasi(Direction.kReverse));

        SysIdRoutine shooterSysId = shooter.getShooterSysIdRoutine();
        autonChooser.addOption("SysID Shooter Dynamic Forward", shooterSysId.dynamic(Direction.kForward));
        autonChooser.addOption("SysID Shooter Dynamic Backwards", shooterSysId.dynamic(Direction.kReverse));
        autonChooser.addOption("SysID Shooter Quasi Forwards", shooterSysId.quasistatic(Direction.kForward));
        autonChooser.addOption("SysID Shooter Quasi Backwards", shooterSysId.quasistatic(Direction.kReverse));

        SysIdRoutine hoodSysId = hood.getHoodSysIdRoutine();
        autonChooser.addOption("SysID Hood Dynamic Forward", hoodSysId.dynamic(Direction.kForward));
        autonChooser.addOption("SysID Hood Dynamic Backwards", hoodSysId.dynamic(Direction.kReverse));
        autonChooser.addOption("SysID Hood Quasi Forwards", hoodSysId.quasistatic(Direction.kForward));
        autonChooser.addOption("SysID Hood Quasi Backwards", hoodSysId.quasistatic(Direction.kReverse));

        SysIdRoutine intakePivotSysId = intake.getPivotSysIdRoutine();
        autonChooser.addOption("SysID Intake Pivot Dynamic Forward", intakePivotSysId.dynamic(Direction.kForward));
        autonChooser.addOption("SysID Intake Pivot Dynamic Backwards", intakePivotSysId.dynamic(Direction.kReverse));
        autonChooser.addOption("SysID Intake Pivot Quasi Forwards", intakePivotSysId.quasistatic(Direction.kForward));
        autonChooser.addOption("SysID Intake Pivot Quasi Backwards", intakePivotSysId.quasistatic(Direction.kReverse));

        SysIdRoutine spindexerSysId = spindexer.getSysIdRoutine();
        autonChooser.addOption("SysID Spindexer Dynamic Forward", spindexerSysId.dynamic(Direction.kForward));
        autonChooser.addOption("SysID Spindexer Dynamic Backwards", spindexerSysId.dynamic(Direction.kReverse));
        autonChooser.addOption("SysID Spindexer Quasi Forwards", spindexerSysId.quasistatic(Direction.kForward));
        autonChooser.addOption("SysID Spindexer Quasi Backwards", spindexerSysId.quasistatic(Direction.kReverse));

        SysIdRoutine handoffSysId = handoff.getSysIdRoutine();
        autonChooser.addOption("SysID Handoff Forward", handoffSysId.dynamic(Direction.kForward));
        autonChooser.addOption("SysID Handoff Backwards", handoffSysId.dynamic(Direction.kReverse));
        autonChooser.addOption("SysID Handoff Forwards", handoffSysId.quasistatic(Direction.kForward));
        autonChooser.addOption("SysID Handoff Backwards", handoffSysId.quasistatic(Direction.kReverse));

    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}