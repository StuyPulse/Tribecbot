/************************ PROJECT 2026 ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.intake.IntakeIntake;
import com.stuypulse.robot.commands.intake.IntakeStow;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.climber.Climber;
import com.stuypulse.robot.subsystems.feeder.Feeder;
import com.stuypulse.robot.subsystems.hoodedshooter.HoodedShooter;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.spindexer.Spindexer;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.turret.Turret;
import com.stuypulse.robot.commands.turret.TurretFerry;
import com.stuypulse.robot.commands.turret.TurretIdle;
import com.stuypulse.robot.commands.turret.TurretSeed;
import com.stuypulse.robot.commands.turret.TurretShoot;
import com.stuypulse.robot.commands.turret.TurretZero;
import com.stuypulse.robot.subsystems.vision.LimelightVision;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);

    // Subsystem
    private final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
    private final Climber climber = Climber.getInstance();
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
                    .andThen(new FeederFeed().onlyIf(() -> hoodedShooter.isShooterAtTolerance())
                        .alongWith(new WaitUntilCommand(() -> feeder.atTolerance()))
                            .andThen(new SpindexerRun().onlyIf(() -> hoodedShooter.isShooterAtTolerance()))
                    )
            )
            .onFalse(new SpindexerStop()
                // .alongWith(new HoodedShooterStow())
                .alongWith(new FeederStop()));

        driver.getBottomButton()
            .onTrue(new TurretShoot());

        driver.getLeftButton()
            .whileTrue(new HoodedShooterShoot())
            .onFalse(new HoodedShooterFerry());
        // driver.getDPadRight()
        //     .whileTrue(
        //         new SwerveXMode().alongWith(
        //             new HoodedShooterShoot().alongWith(
        //                 new TurretShoot()).alongWith(
        //                     new WaitUntilCommand(() -> hoodedShooter.isHoodAtTolerance())).alongWith(
        //                     new WaitUntilCommand(() -> hoodedShooter.isShooterAtTolerance())).alongWith(
        //                     new WaitUntilCommand(() -> turret.atTargetAngle())).andThen(
        //                         new SpindexerRun().alongWith(new FeederFeed()))))
        //     .onFalse(
        //         new HoodedShooterStow().alongWith(
        //         new TurretHoodAlignToTarget().alongWith(
        //         new SpindexerRun().alongWith(
        //         new FeederStop())))
        //     );

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

        /**
        // Climb Align
        driver.getTopButton()
            .whileTrue(SwerveClimbAlign());

        // Left Corner Shoot
        driver.getLeftButton()
            .whileTrue(
                new SwerveXMode().alongWith(
                    new HoodedShooterLeftCorner().alongWith(
                        new TurretLeftCorner()).alongWith(
                            new WaitUntilCommand(() -> hoodedShooter.isHoodAtTolerance())).alongWith(
                            new WaitUntilCommand(() -> hoodedShooter.isShooterAtTolerance())).alongWith(
                            new WaitUntilCommand(() -> turret.atTargetAngle())).andThen(
                                new SpindexerRun().alongWith(new FeederFeed()))))
            .onFalse(
                new HoodedShooterStow().alongWith(
                new TurretHoodAlignToTarget().alongWith(
                new SpindexerRun().alongWith(
                new FeederStop())))
            );

        // Right Corner Shoot
        driver.getRightButton()
            .whileTrue(
                new SwerveXMode().alongWith(
                    new HoodedShooterRightCorner().alongWith(
                        new TurretRightCorner()).alongWith(
                            new WaitUntilCommand(() -> hoodedShooter.isHoodAtTolerance())).alongWith(
                            new WaitUntilCommand(() -> hoodedShooter.isShooterAtTolerance())).alongWith(
                            new WaitUntilCommand(() -> turret.atTargetAngle())).andThen(
                                new SpindexerRun().alongWith(new FeederFeed()))))
            .onFalse(
                new HoodedShooterStow().alongWith(
                new TurretHoodAlignToTarget().alongWith(
                new SpindexerRun().alongWith(
                new FeederStop())))
            );

        // Hub Shoot
        driver.getBottomButton()
            .whileTrue(
                new SwerveXMode().alongWith(
                    new HoodedShooterHub().alongWith(
                        new TurretHub()).alongWith(
                            new WaitUntilCommand(() -> hoodedShooter.isHoodAtTolerance())).alongWith(
                            new WaitUntilCommand(() -> hoodedShooter.isShooterAtTolerance())).alongWith(
                            new WaitUntilCommand(() -> turret.atTargetAngle())).andThen(
                                new SpindexerRun().alongWith(new FeederFeed()))))
            .onFalse(
                new HoodedShooterStow().alongWith(
                new TurretHoodAlignToTarget().alongWith(
                new SpindexerRun().alongWith(
                new FeederStop())))
            );

        // Intake On
        driver.getLeftTriggerButton()
            .onTrue(new IntakeIntake());

        // Intake Off
        driver.getRightTriggerButton()
            .onTrue(new IntakeStop());

        // Climb Down Placeholder
        driver.getLeftBumper()
            .onTrue(new BuzzController(driver));

        // Climb Up Placeholder
        driver.getRightBumper()
            .onTrue(new BuzzController(driver));

        // Reset Heading
        driver.getDPadUp()
            .onTrue(new SwerveResetHeading());

        // Ferry In Place
        driver.getDPadLeft()
            .whileTrue(
                new SwerveXMode().alongWith(
                    new HoodedShooterFerry().alongWith(
                        new TurretFerry()).alongWith(
                            new WaitUntilCommand(() -> hoodedShooter.isHoodAtTolerance())).alongWith(
                            new WaitUntilCommand(() -> hoodedShooter.isShooterAtTolerance())).alongWith(
                            new WaitUntilCommand(() -> turret.atTargetAngle())).andThen(
                                new SpindexerRun().alongWith(new FeederFeed()))))
            .onFalse(
                new HoodedShooterStow().alongWith(
                new TurretHoodAlignToTarget().alongWith(
                new SpindexerRun().alongWith(
                new FeederStop())))
            );

        // Score In Place
        driver.getDPadRight()
            .whileTrue(
                new SwerveXMode().alongWith(
                    new HoodedShooterShoot().alongWith(
                        new TurretShoot()).alongWith(
                            new WaitUntilCommand(() -> hoodedShooter.isHoodAtTolerance())).alongWith(
                            new WaitUntilCommand(() -> hoodedShooter.isShooterAtTolerance())).alongWith(
                            new WaitUntilCommand(() -> turret.atTargetAngle())).andThen(
                                new SpindexerRun().alongWith(new FeederFeed()))))
            .onFalse(
                new HoodedShooterStow().alongWith(
                new TurretHoodAlignToTarget().alongWith(
                new SpindexerRun().alongWith(
                new FeederStop())))
            );

        // Unjam
        driver.getDPadDown()
            .whileTrue(
                new HoodedShooterReverse().alongWith(
                    new FeederReverse().alongWith(
                        new IntakeOutake())))
            .onFalse(
                new HoodedShooterStow().alongWith(
                new TurretHoodAlignToTarget().alongWith(
                new SpindexerRun().alongWith(
                new FeederStop().alongWith(
                new IntakeStop()))))
            );

        driver.getLeftMenuButton()
            .onTrue(
                new HoodedShooterFerry().alongWith(
                        new TurretFerry()).alongWith(
                            new WaitUntilCommand(() -> hoodedShooter.isHoodAtTolerance())).alongWith(
                            new WaitUntilCommand(() -> hoodedShooter.isShooterAtTolerance())).alongWith(
                            new WaitUntilCommand(() -> turret.atTargetAngle())).andThen(
                                new SpindexerRun().alongWith(new FeederFeed())))
            .onFalse(
                new HoodedShooterStow().alongWith(
                new TurretHoodAlignToTarget().alongWith(
                new SpindexerRun().alongWith(
                new FeederStop())))
            );

        driver.getRightMenuButton()
            .onTrue(
                new HoodedShooterFerry().alongWith(
                        new TurretFerry()).alongWith(
                            new WaitUntilCommand(() -> hoodedShooter.isHoodAtTolerance())).alongWith(
                            new WaitUntilCommand(() -> hoodedShooter.isShooterAtTolerance())).alongWith(
                            new WaitUntilCommand(() -> turret.atTargetAngle())).andThen(
                                new SpindexerRun().alongWith(new FeederFeed())))
            .onFalse(
                new HoodedShooterStow().alongWith(
               new TurretHoodAlignToTarget().alongWith(
                new SpindexerRun().alongWith(
                new FeederStop())))
            );

        **/

    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public void configureSysids() {
        SysIdRoutine intakePivotSysId = intake.getPivotSysIdRoutine();
        autonChooser.addOption("SysID Intake Pivot Dynamic Forward", intakePivotSysId.dynamic(Direction.kForward));
        autonChooser.addOption("SysID Intake Pivot Dynamic Backwards", intakePivotSysId.dynamic(Direction.kReverse));
        autonChooser.addOption("SysID Intake Pivot Quasi Forwards", intakePivotSysId.quasistatic(Direction.kForward));
        autonChooser.addOption("SysID Intake Pivot Quasi Backwards", intakePivotSysId.quasistatic(Direction.kReverse));

        SysIdRoutine intakeRollerSysId = intake.getRollerSysIdRoutine();
        autonChooser.addOption("SysID Intake Roller Dynamic Forward", intakeRollerSysId.dynamic(Direction.kForward));
        autonChooser.addOption("SysID Intake Roller Dynamic Backwards", intakeRollerSysId.dynamic(Direction.kReverse));
        autonChooser.addOption("SysID Intake Roller Quasi Forwards", intakeRollerSysId.quasistatic(Direction.kForward));
        autonChooser.addOption("SysID Intake Roller Quasi Backwards", intakeRollerSysId.quasistatic(Direction.kReverse));

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