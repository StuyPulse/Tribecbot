/************************ PROJECT 2026 ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.feeder.SetFeederForward;
import com.stuypulse.robot.commands.feeder.SetFeederStop;
import com.stuypulse.robot.commands.hoodedshooter.HoodedShooterFerry;
import com.stuypulse.robot.commands.hoodedshooter.HoodedShooterShoot;
import com.stuypulse.robot.commands.intake.IntakeIntake;
import com.stuypulse.robot.commands.intake.IntakeStow;
import com.stuypulse.robot.commands.spindexer.SetSpindexerForward;
import com.stuypulse.robot.commands.spindexer.SetSpindexerStop;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveResetHeading;
import com.stuypulse.robot.commands.turret.TurretFerry;
import com.stuypulse.robot.commands.turret.TurretSeed;
import com.stuypulse.robot.commands.turret.TurretShoot;
import com.stuypulse.robot.commands.turret.TurretZero;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.climberhopper.ClimberHopper;
import com.stuypulse.robot.subsystems.feeder.Feeder;
import com.stuypulse.robot.subsystems.hoodedshooter.HoodedShooter;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.spindexer.Spindexer;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.turret.Turret;
import com.stuypulse.robot.subsystems.vision.LimelightVision;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;


public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);

    // Subsystem
    private final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
    private final ClimberHopper climberHopper = ClimberHopper.getInstance();
    private final Feeder feeder = Feeder.getInstance();
    private final HoodedShooter hoodedShooter = HoodedShooter.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Spindexer spindexer = Spindexer.getInstance();
    private final Turret turret = Turret.getInstance();
    private final LimelightVision vision = LimelightVision.getInstance();

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
        //hoodedshooter.setDefaultCommand(new TurretHoodAlignToTarget());
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {
        driver.getBottomButton().onTrue(new IntakeIntake());
        driver.getTopButton().onTrue(new IntakeStow());

        driver.getRightTriggerButton().onTrue(new TurretShoot());

        driver.getLeftTriggerButton().onTrue(new TurretFerry());

        driver.getDPadDown()
            .onTrue(new TurretSeed());

        driver.getDPadUp()
            .onTrue(new SwerveResetHeading());
            
        // driver.getDPadUp().onTrue(new TurretAnalog(driver));

        // SCORING ROUTINE
        driver.getTopButton()
            .whileTrue(new TurretZero()
                .alongWith(new HoodedShooterShoot())
                    .alongWith(new WaitUntilCommand(() -> hoodedShooter.isShooterAtTolerance()))
                    .andThen(new SetFeederForward().onlyIf(() -> hoodedShooter.isShooterAtTolerance())
                        .alongWith(new WaitUntilCommand(() -> feeder.atTolerance()))
                            .andThen(new SetSpindexerForward().onlyIf(() -> hoodedShooter.isShooterAtTolerance()))
                    )
            )
            .onFalse(new SetSpindexerStop()
                // .alongWith(new HoodedShooterStow())
                .alongWith(new SetFeederStop()));

        driver.getBottomButton()
            .onTrue(new TurretShoot());

        driver.getLeftButton()
            .whileTrue(new HoodedShooterShoot())
            .onFalse(new HoodedShooterFerry());
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

        // SysIdRoutine shooterSysId = shooter.getShooterSysIdRoutine();
        // autonChooser.addOption("SysID Shooter Dynamic Forward", shooterSysId.dynamic(Direction.kForward));
        // autonChooser.addOption("SysID Shooter Dynamic Backwards", shooterSysId.dynamic(Direction.kReverse));
        // autonChooser.addOption("SysID Shooter Quasi Forwards", shooterSysId.quasistatic(Direction.kForward));
        // autonChooser.addOption("SysID Shooter Quasi Backwards", shooterSysId.quasistatic(Direction.kReverse));

        // SysIdRoutine hoodSysId = hood.getHoodSysIdRoutine();
        // autonChooser.addOption("SysID Hood Dynamic Forward", hoodSysId.dynamic(Direction.kForward));
        // autonChooser.addOption("SysID Hood Dynamic Backwards", hoodSysId.dynamic(Direction.kReverse));
        // autonChooser.addOption("SysID Hood Quasi Forwards", hoodSysId.quasistatic(Direction.kForward));
        // autonChooser.addOption("SysID Hood Quasi Backwards", hoodSysId.quasistatic(Direction.kReverse));

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

        SysIdRoutine feederSysId = feeder.getSysIdRoutine();
        autonChooser.addOption("SysID Feeder Forward", feederSysId.dynamic(Direction.kForward));
        autonChooser.addOption("SysID Feeder Backwards", feederSysId.dynamic(Direction.kReverse));
        autonChooser.addOption("SysID Feeder Forwards", feederSysId.quasistatic(Direction.kForward));
        autonChooser.addOption("SysID Feeder Backwards", feederSysId.quasistatic(Direction.kReverse));

    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}