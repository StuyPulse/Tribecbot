/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot;

import com.stuypulse.robot.commands.BuzzController;
import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.auton.regular.DepotAuton;
import com.stuypulse.robot.commands.auton.regular.LeftTwoCycle;
import com.stuypulse.robot.commands.auton.regular.RightTwoCycle;
import com.stuypulse.robot.commands.auton.regular.SecretRightTwoCycle;
import com.stuypulse.robot.commands.handoff.HandoffReverse;
import com.stuypulse.robot.commands.handoff.HandoffRun;
import com.stuypulse.robot.commands.handoff.HandoffStop;
import com.stuypulse.robot.commands.hood.SeedHoodRelativeEncoderAtUpperHardstop;
import com.stuypulse.robot.commands.intake.IntakeDeploy;
import com.stuypulse.robot.commands.intake.IntakeOuttake;
import com.stuypulse.robot.commands.intake.IntakeRunRollers;
import com.stuypulse.robot.commands.intake.IntakeSetState;
import com.stuypulse.robot.commands.intake.IntakeStopRollers;
import com.stuypulse.robot.commands.intake.ZeroPivotDeployed;
import com.stuypulse.robot.commands.intake.ZeroPivotStowed;
import com.stuypulse.robot.commands.spindexer.SpindexerReverse;
import com.stuypulse.robot.commands.spindexer.SpindexerRun;
import com.stuypulse.robot.commands.spindexer.SpindexerStop;
import com.stuypulse.robot.commands.superstructure.SuperstructureFOTM;
import com.stuypulse.robot.commands.superstructure.SuperstructureInterpolation;
import com.stuypulse.robot.commands.superstructure.SuperstructureKB;
import com.stuypulse.robot.commands.superstructure.SuperstructureLeftCorner;
import com.stuypulse.robot.commands.superstructure.SuperstructureRightCorner;
import com.stuypulse.robot.commands.superstructure.SuperstructureSOTM;
import com.stuypulse.robot.commands.superstructure.SuperstructureShoot;
import com.stuypulse.robot.commands.superstructure.SuperstructureStow;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveDriveFOTM;
import com.stuypulse.robot.commands.swerve.SwerveDriveSOTM;
import com.stuypulse.robot.commands.swerve.SwerveResetHeading;
import com.stuypulse.robot.commands.swerve.SwerveXMode;
import com.stuypulse.robot.commands.turret.SeedTurret;
import com.stuypulse.robot.commands.turret.ZeroTurret;
import com.stuypulse.robot.commands.vision.EnableBackLimelight;
import com.stuypulse.robot.commands.vision.EnableLeftLimelight;
import com.stuypulse.robot.commands.vision.EnableRightLimelight;
import com.stuypulse.robot.commands.vision.ResetLimelightIMU;
import com.stuypulse.robot.commands.vision.SetIMUMode;
import com.stuypulse.robot.commands.vision.SetMegaTagMode;
import com.stuypulse.robot.commands.vision.WhitelistAllTagsForAllCameras;
import com.stuypulse.robot.commands.vision.WhitelistOutpostTags;
import com.stuypulse.robot.commands.vision.WhitelistTowerTags;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.handoff.Handoff;
import com.stuypulse.robot.subsystems.handoff.Handoff.HandoffState;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.intake.Intake.RollerState;
import com.stuypulse.robot.subsystems.spindexer.Spindexer;
import com.stuypulse.robot.subsystems.spindexer.Spindexer.SpindexerState;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import com.stuypulse.robot.subsystems.superstructure.hood.Hood;
import com.stuypulse.robot.subsystems.superstructure.shooter.Shooter;
import com.stuypulse.robot.subsystems.superstructure.turret.Turret;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.vision.LimelightVision;
import com.stuypulse.robot.subsystems.vision.LimelightVision.MegaTagMode;
import com.stuypulse.robot.util.PathUtil.AutonConfig;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;
import com.stuypulse.stuylib.network.SmartBoolean;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;


public class RobotContainer {
    public interface EnabledSubsystems {
        SmartBoolean SWERVE = new SmartBoolean("Enabled Subsystems/Swerve Is Enabled", true);
        SmartBoolean TURRET = new SmartBoolean("Enabled Subsystems/Turret Is Enabled", true);
        SmartBoolean HANDOFF = new SmartBoolean("Enabled Subsystems/Handoff Is Enabled", true);
        SmartBoolean INTAKE = new SmartBoolean("Enabled Subsystems/Intake Is Enabled", true);
        SmartBoolean SPINDEXER = new SmartBoolean("Enabled Subsystems/Spindexer Is Enabled", true);
        SmartBoolean HOOD = new SmartBoolean("Enabled Subsystems/Hood Is Enabled", true);
        SmartBoolean SHOOTER = new SmartBoolean("Enabled Subsystems/Shooter Is Enabled", true);

        SmartBoolean BACK_LIMELIGHT = new SmartBoolean("Enabled Subsystems/Back Limelight Is Enabled", true);
        SmartBoolean LEFT_LIMELIGHT = new SmartBoolean("Enabled Subsystems/Left Limelight Is Enabled", true);
        SmartBoolean RIGHT_LIMELIGHT = new SmartBoolean("Enabled Subsystems/Right Limelight Is Enabled", true);
    }

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);

    // Subsystem
    private final Handoff handoff = Handoff.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Spindexer spindexer = Spindexer.getInstance();
    
    private final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
    private final LimelightVision vision = LimelightVision.getInstance();
    private final Turret turret = Turret.getInstance();

    private final Superstructure superstructure = Superstructure.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Hood hood = Hood.getInstance();

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot container
    public RobotContainer() {
        swerve.configureAutoBuilder();
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();
        configureSysids();
        configureElasticButtons();

        SmartDashboard.putData("Field", Field.FIELD2D);
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new SwerveDriveDrive(driver));
    }

    /***************/
    /*** ELASTIC ***/
    /***************/

    private void configureElasticButtons() {
        SmartDashboard.putData("Robot/Zero Pivot Encoder at Lower Limit (Deployed)", new ZeroPivotDeployed().ignoringDisable(true));
        SmartDashboard.putData("Robot/Zero Pivot Encoder at Upper Limit (Stowed)", new ZeroPivotStowed().ignoringDisable(true));
        SmartDashboard.putData("Robot/Zero Turret Encoders", new ZeroTurret().ignoringDisable(true));
        SmartDashboard.putData("Robot/Seed Turret", new SeedTurret().ignoringDisable(true));
        SmartDashboard.putData("Robot/Seed Hood Relative Encoder At Upper Hardstop", new SeedHoodRelativeEncoderAtUpperHardstop().ignoringDisable(true));
        SmartDashboard.putData("Robot/Set Megatag 1", new SetMegaTagMode(MegaTagMode.MEGATAG1).ignoringDisable(true));
        SmartDashboard.putData("Robot/Set Megatag 2", new SetMegaTagMode(MegaTagMode.MEGATAG2).ignoringDisable(true));

        SmartDashboard.putData("Robot/Set Left LL PF", new EnableLeftLimelight());
        SmartDashboard.putData("Robot/Set Right LL PF", new EnableRightLimelight());
        SmartDashboard.putData("Robot/Set Back LL PF", new EnableBackLimelight());

        SmartDashboard.putData("Robot/WL Outpost Tags Left-Camera", new WhitelistOutpostTags("limelight-left").ignoringDisable(true));
        // SmartDashboard.putData("Robot/Reset WL Left-Camera", new WhitelistAllTags("limelight-left").ignoringDisable(true));

        SmartDashboard.putData("Robot/WL Tower Tags Right-Camera", new WhitelistTowerTags("limelight-right").ignoringDisable(true));
        // SmartDashboard.putData("Robot/Reset WL Right-Camera", new WhitelistAllTags("limelight-right").ignoringDisable(true));

        SmartDashboard.putData("Robot/Whitelist All Cameras", new WhitelistAllTagsForAllCameras());

        SmartDashboard.putData("Robot/Handoff Reverse", 
            new ConditionalCommand(
                new HandoffReverse().andThen(new WaitCommand(0.25)).andThen(new HandoffRun()), 
                new HandoffReverse().andThen(new WaitCommand(0.25).andThen(new HandoffStop())),
                () -> handoff.getState() == HandoffState.FORWARD));

        SmartDashboard.putData("Robot/Intake Reverse", new IntakeSetState(RollerState.OUTTAKE));

        SmartDashboard.putData("Robot/Spindexer Reverse", 
            new ConditionalCommand(
                new SpindexerReverse().andThen(new WaitCommand(1)).andThen(new SpindexerRun()), 
                new SpindexerReverse().andThen(new WaitCommand(1).andThen(new SpindexerStop())),
                () -> spindexer.getState() == SpindexerState.FORWARD));
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {
        // Scoring Routine 
        driver.getTopButton()
            .whileTrue(new SwerveXMode())
            .whileTrue(new RepeatCommand(new BuzzController(driver).onlyWhile(() -> !vision.hasData())))
            .onTrue(new WaitUntilCommand(() -> spindexer.canStartIntakeRollers()).andThen(new IntakeRunRollers()))
            .whileTrue(new SuperstructureShoot()
                    .alongWith(new WaitUntilCommand(() -> superstructure.getState() == SuperstructureState.SHOOT && superstructure.atTolerance()))
                        .andThen(new HandoffRun())
                        .alongWith(new WaitUntilCommand(() -> handoff.getState() == HandoffState.FORWARD && handoff.atTolerance()))
                            .andThen(new SpindexerRun()))
            .onFalse(new SpindexerStop()
                    .alongWith(new SuperstructureStow())
                    .alongWith(new HandoffStop()));

        // Intake Stow
        // driver.getLeftTriggerButton()
        //     .onTrue(new IntakeStow());

        // Intake Deploy
        driver.getRightTriggerButton()
            .onTrue(new IntakeDeploy());
            // .onTrue(new SuperstructureStow()                    
            //         .alongWith(new SpindexerStop()) //TODO: test this logic
            //         .alongWith(new HandoffStop())); // TURNS OFF SOTM
        
        // Reset Heading
        driver.getDPadUp()
            .onTrue(new SwerveResetHeading())
            .onTrue(new ResetLimelightIMU())
            .onFalse(new SetIMUMode(0));   

        // Stop Rollers
        driver.getLeftBumper()
            .onTrue(new IntakeDeploy().andThen(new IntakeStopRollers()));

        driver.getRightBumper()
            .whileTrue(new IntakeOuttake())
            .onFalse(new IntakeRunRollers());

        // Ferrying In Place
        driver.getDPadRight()
            .onTrue(new SuperstructureStow()
                .alongWith(new HandoffStop())
                .alongWith(new SpindexerStop()));
        //     .whileTrue(new SwerveXMode())
        //     .onTrue(new IntakeRunRollers())
        //     .whileTrue(new SuperstructureInterpolation() // CURRENTLY, THIS PROVES THE FERRYING STATE IS BLOCKING SPINDEXER AND HANDOFF SOMEWHERE IN THE CODE
        //             .alongWith(new WaitUntilCommand(() -> superstructure.getState() == SuperstructureState.INTERPOLATION && superstructure.atTolerance()))
        //                 .andThen(new HandoffRun())
        //             .alongWith(new WaitUntilCommand(() -> handoff.getState() == HandoffState.FORWARD && handoff.atTolerance()))
        //                 .andThen(new SpindexerRun()))
        //     .onFalse(new SpindexerStop()
        //         .alongWith(new SuperstructureStow())
        //         .alongWith(new HandoffStop()));
        
        // SOTM
        driver.getRightMenuButton()
            .whileTrue(new RepeatCommand(new BuzzController(driver).onlyWhile(() -> !vision.hasData())))
            .onTrue(new IntakeRunRollers())
            .onTrue(new ConditionalCommand(
                new ParallelCommandGroup(
                    new SuperstructureStow(), 
                    new SpindexerStop(),
                    new HandoffStop()
                ),
                new ParallelCommandGroup(
                    new SuperstructureSOTM().alongWith(new WaitUntilCommand(() -> superstructure.atTolerance()))
                        .andThen(new HandoffRun())
                    .alongWith(new WaitUntilCommand(() -> handoff.atTolerance()))
                        .andThen(new SpindexerRun()),
                    new SwerveDriveSOTM(driver)
                ),
                () -> superstructure.getState() == SuperstructureState.SOTM
            ));

        // FOTM
        driver.getLeftMenuButton()
            .onTrue(new IntakeRunRollers())
            .onTrue(new ConditionalCommand(
                new ParallelCommandGroup(
                    new SuperstructureStow(),
                    new SpindexerStop(),
                    new HandoffStop()
                ),
                new ParallelCommandGroup(
                    new SuperstructureFOTM().alongWith(new WaitUntilCommand(() -> superstructure.atTolerance()))
                        .andThen(new HandoffRun())
                    .alongWith(new WaitUntilCommand(() -> handoff.atTolerance()))
                        .andThen(new SpindexerRun()),
                    new SwerveDriveFOTM(driver)
                ),
                () -> superstructure.getState() == SuperstructureState.FOTM
            ));

        driver.getDPadDown()
            .whileTrue(new SwerveXMode());

//--------------------------------------------------------------------------------------------------------------------------\\

        // Manual Left Corner Scoring
        driver.getLeftButton()
            .whileTrue(new SwerveXMode())
            .onTrue(new IntakeRunRollers())
            .whileTrue(new SuperstructureLeftCorner().alongWith(new WaitUntilCommand(() -> superstructure.atTolerance()))
                .andThen(new HandoffRun()).alongWith(new WaitUntilCommand(() -> handoff.getState() == HandoffState.FORWARD && handoff.atTolerance())
                .andThen(new SpindexerRun())))
            .onFalse(new SuperstructureStow().alongWith(new SpindexerStop()).alongWith(new HandoffStop()));

        // Manual Right Corner Scoring
        driver.getRightButton()
            .whileTrue(new SwerveXMode())
            .onTrue(new IntakeRunRollers())
            .whileTrue(new SuperstructureRightCorner().alongWith(new WaitUntilCommand(() -> superstructure.atTolerance()))
                .andThen(new HandoffRun()).alongWith(new WaitUntilCommand(() -> handoff.getState() == HandoffState.FORWARD && handoff.atTolerance())
                .andThen(new SpindexerRun())))
            .onFalse(new SuperstructureStow().alongWith(new SpindexerStop()).alongWith(new HandoffStop()));

        // Manual KB Distance Scoring
        driver.getBottomButton()
            .whileTrue(new SwerveXMode())
            .onTrue(new IntakeRunRollers())
            .whileTrue(new SuperstructureKB().alongWith(new WaitUntilCommand(() -> superstructure.atTolerance()))
                .andThen(new HandoffRun()).alongWith(new WaitUntilCommand(() -> handoff.getState() == HandoffState.FORWARD && handoff.atTolerance())
                .andThen(new SpindexerRun())))
            .onFalse(new SuperstructureStow().alongWith(new SpindexerStop()).alongWith(new HandoffStop()));
        
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {

        autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());

        // DEPOT
        AutonConfig DEPOT_AUTON = new AutonConfig("Depot Auton", DepotAuton::new, 
        "Left Bump To Depot", "Depot To Tower Left");
        DEPOT_AUTON.register(autonChooser);

        // ONE CYCLES
        // AutonConfig LEFT_ONE_CYCLE = new AutonConfig("Left One Cycle", LeftOneCycle::new,  
        // "Left Trench To NZ", "Left NZ To Score");
        // LEFT_ONE_CYCLE.register(autonChooser);

        // AutonConfig RIGHT_ONE_CYCLE = new AutonConfig("Right One Cycle", RightOneCycle::new,  
        // "Right Trench To NZ", "Right NZ To Score");
        // RIGHT_ONE_CYCLE.register(autonChooser);

        // TWO CYCLES
        AutonConfig LEFT_TWO_CYCLE = new AutonConfig("Left Two Cycle", LeftTwoCycle::new,  
        "Left Trench To NZ", "Left NZ To Score", "Left Score To Score");
        LEFT_TWO_CYCLE.register(autonChooser);

        AutonConfig RIGHT_TWO_CYCLE = new AutonConfig("Right Two Cycle", RightTwoCycle::new,  
        "Right Trench To NZ", "Right NZ To Score", "Right Score To Score");
        RIGHT_TWO_CYCLE.register(autonChooser);

        AutonConfig SECRET_RIGHT_TWO_CYCLE = new AutonConfig("Secret Right Two Cycle", SecretRightTwoCycle::new,  
        "Secret Right Trench To NZ", "Secret Right NZ To Score", "Right Score To Score");
        SECRET_RIGHT_TWO_CYCLE.register(autonChooser);

        SmartDashboard.putData("Autonomous", autonChooser);

    }

    public void configureSysids() {

        // autonChooser.addOption("SysID Module Translation Dynamic Forwards", swerve.sysIdDynamic(Direction.kForward));
        // autonChooser.addOption("SysID Module Translation Dynamic Backwards", swerve.sysIdDynamic(Direction.kReverse));
        // autonChooser.addOption("SysID Module Translation Quasi Forwards", swerve.sysIdQuasistatic(Direction.kForward));
        // autonChooser.addOption("SysID Module Translation Quasi Backwards", swerve.sysIdQuasistatic(Direction.kReverse)); 

        // autonChooser.addOption("SysID Rotation Translation Dynamic Forwards", swerve.sysidRotationDynamic(Direction.kForward));
        // autonChooser.addOption("SysID Rotation Translation Dynamic Backwards", swerve.sysidRotationDynamic(Direction.kReverse));
        // autonChooser.addOption("SysID Rotation Translation Quasi Forwards", swerve.sysidRotationQuasiStatic(Direction.kForward));
        // autonChooser.addOption("SysID Rotation Translation Quasi Backwards", swerve.sysidRotationQuasiStatic(Direction.kReverse)); 
        
        // autonChooser.addOption("SysID Turret Dynamic Forwards", turret.getSysIdRoutine().dynamic(Direction.kForward));
        // autonChooser.addOption("SysID Turret Dynamic Reverse", turret.getSysIdRoutine().dynamic(Direction.kReverse));
        // autonChooser.addOption("SysID Turret Quasistatic Forwards", turret.getSysIdRoutine().quasistatic(Direction.kForward));
        // autonChooser.addOption("SysID Turret Quasistatic Reverse", turret.getSysIdRoutine().quasistatic(Direction.kReverse));

        // autonChooser.addOption("SysID Module Rotation Dynamic Forwards", swerve.sysIdRotDynamic(Direction.kForward));
        // autonChooser.addOption("SysID Module Rotation Dynamic Backwards", swerve.sysIdRotDynamic(Direction.kReverse));
        // autonChooser.addOption("SysID Module Rotation Quasi Forwards", swerve.sysIdRotQuasi(Direction.kForward));
        // autonChooser.addOption("SysID Module Rotation Quasi Backwards", swerve.sysIdRotQuasi(Direction.kReverse));

        // SysIdRoutine shooterSysId = shooter.getShooterSysIdRoutine();
        // autonChooser.addOption("SysID Shooter Dynamic Forwards", shooterSysId.dynamic(Direction.kForward));
        // autonChooser.addOption("SysID Shooter Dynamic Backwards", shooterSysId.dynamic(Direction.kReverse));
        // autonChooser.addOption("SysID Shooter Quasi Forwards", shooterSysId.quasistatic(Direction.kForward));
        // autonChooser.addOption("SysID Shooter Quasi Backwards", shooterSysId.quasistatic(Direction.kReverse));

        // SysIdRoutine hoodSysId = hood.getHoodSysIdRoutine();
        // autonChooser.addOption("SysID Hood Dynamic Forwards", hoodSysId.dynamic(Direction.kForward));
        // autonChooser.addOption("SysID Hood Dynamic Backwards", hoodSysId.dynamic(Direction.kReverse));
        // autonChooser.addOption("SysID Hood Quasi Forwards", hoodSysId.quasistatic(Direction.kForward));
        // autonChooser.addOption("SysID Hood Quasi Backwards", hoodSysId.quasistatic(Direction.kReverse));

        // SysIdRoutine intakePivotSysId = intake.getPivotSysIdRoutine();
        // autonChooser.addOption("SysID Intake Pivot Dynamic Forwards", intakePivotSysId.dynamic(Direction.kForward));
        // autonChooser.addOption("SysID Intake Pivot Dynamic Backwards", intakePivotSysId.dynamic(Direction.kReverse));
        // autonChooser.addOption("SysID Intake Pivot Quasi Forwards", intakePivotSysId.quasistatic(Direction.kForward));
        // autonChooser.addOption("SysID Intake Pivot Quasi Backwards", intakePivotSysId.quasistatic(Direction.kReverse));

        // SysIdRoutine spindexerSysId = spindexer.getSysIdRoutine();
        // autonChooser.addOption("SysID Spindexer Dynamic Forwards", spindexerSysId.dynamic(Direction.kForward));
        // autonChooser.addOption("SysID Spindexer Dynamic Backwards", spindexerSysId.dynamic(Direction.kReverse));
        // autonChooser.addOption("SysID Spindexer Quasi Forwards", spindexerSysId.quasistatic(Direction.kForward));
        // autonChooser.addOption("SysID Spindexer Quasi Backwards", spindexerSysId.quasistatic(Direction.kReverse));

        // // Wheel Radius Characterization
        // autonChooser.addOption("Wheel Characterization", new SwerveWheelRadiusCharacterization());

        // SysIdRoutine handoffSysId = handoff.getSysIdRoutine();
        // autonChooser.addOption("SysID Handoff Dynamic Forward", handoffSysId.dynamic(Direction.kForward));
        // autonChooser.addOption("SysID Handoff Dynamic Backwards", handoffSysId.dynamic(Direction.kReverse));
        // autonChooser.addOption("SysID Handoff Quasi Forwards", handoffSysId.quasistatic(Direction.kForward));
        // autonChooser.addOption("SysID Handoff Quasi Backwards", handoffSysId.quasistatic(Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        if (autonChooser.getSelected() == null) {
            return new DoNothingAuton();
        }
        else {
            return autonChooser.getSelected();
        }
    }

    public void periodic() {
        //TODO: get from energy util after testing
        double totalCurrentDraw =   handoff.getCurrentDraw() +
                                    intake.getCurrentDraw() +
                                    spindexer.getCurrentDraw() +
                                    superstructure.getCurrentDraw() +
                                    swerve.getTotalDriveSupplyCurrent() +
                                    swerve.getTotalSteerSupplyCurrent();      
        SmartDashboard.putNumber("Robot/Total Current Draw", totalCurrentDraw);
    }
}