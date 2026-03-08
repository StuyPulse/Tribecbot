/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.auton.poaching.BottomOneCyclePoach;
import com.stuypulse.robot.commands.auton.poaching.BottomTwoCyclePoach;
import com.stuypulse.robot.commands.auton.poaching.TopOneCyclePoach;
import com.stuypulse.robot.commands.auton.poaching.TopTwoCyclePoach;
import com.stuypulse.robot.commands.auton.regular.BottomTwoCycle;
import com.stuypulse.robot.commands.auton.regular.DepotAuton;
import com.stuypulse.robot.commands.auton.regular.EightFuel;
import com.stuypulse.robot.commands.auton.regular.TopTwoCycle;
import com.stuypulse.robot.commands.climberhopper.ClimberDown;
// import com.stuypulse.robot.commands.auton.test.BoxTest;
import com.stuypulse.robot.commands.climberhopper.ClimberOverrideDown;
import com.stuypulse.robot.commands.climberhopper.ClimberOverrideStop;
import com.stuypulse.robot.commands.climberhopper.ClimberOverrideUp;
import com.stuypulse.robot.commands.handoff.HandoffConditionalCommand;
import com.stuypulse.robot.commands.climberhopper.ClimberUp;
import com.stuypulse.robot.commands.climberhopper.HopperDown;
import com.stuypulse.robot.commands.handoff.HandoffRun;
import com.stuypulse.robot.commands.handoff.HandoffStop;
import com.stuypulse.robot.commands.hood.ZeroHoodEncoderAtUpperHardstop;
import com.stuypulse.robot.commands.intake.IntakeDeploy;
import com.stuypulse.robot.commands.intake.IntakeDigestion;
import com.stuypulse.robot.commands.intake.IntakeOuttake;
import com.stuypulse.robot.commands.intake.IntakeRunRollers;
import com.stuypulse.robot.commands.intake.IntakeStopRollers;
import com.stuypulse.robot.commands.intake.IntakeStow;
import com.stuypulse.robot.commands.intake.ZeroPivotDeployed;
import com.stuypulse.robot.commands.intake.ZeroPivotStowed;
import com.stuypulse.robot.commands.spindexer.SpindexerConditionalCommand;
import com.stuypulse.robot.commands.spindexer.SpindexerRun;
import com.stuypulse.robot.commands.spindexer.SpindexerStop;
import com.stuypulse.robot.commands.superstructure.SuperstructureDefaultCommand;
import com.stuypulse.robot.commands.superstructure.SuperstructureFOTM;
import com.stuypulse.robot.commands.superstructure.SuperstructureFerry;
import com.stuypulse.robot.commands.superstructure.SuperstructureInterpolation;
import com.stuypulse.robot.commands.superstructure.SuperstructureKB;
import com.stuypulse.robot.commands.superstructure.SuperstructureLeftCorner;
import com.stuypulse.robot.commands.superstructure.SuperstructureRightCorner;
import com.stuypulse.robot.commands.superstructure.SuperstructureSOTM;
import com.stuypulse.robot.commands.superstructure.SuperstructureSOTMConditional;
import com.stuypulse.robot.commands.superstructure.SuperstructureShoot;
import com.stuypulse.robot.commands.superstructure.SuperstructureStow;
import com.stuypulse.robot.commands.swerve.SwerveDriveAlignTurretToHub;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveResetHeading;
import com.stuypulse.robot.commands.swerve.SwerveWheelRadiusCharacterization;
import com.stuypulse.robot.commands.swerve.SwerveXMode;
import com.stuypulse.robot.commands.swerve.climbAlign.SwerveClimbAlign;
import com.stuypulse.robot.commands.turret.TurretDefaultCommand;
import com.stuypulse.robot.commands.turret.TurretIdle;
import com.stuypulse.robot.commands.turret.TurretShoot;
import com.stuypulse.robot.commands.turret.ZeroTurret;
import com.stuypulse.robot.commands.vision.ResetLimelightIMU;
import com.stuypulse.robot.commands.vision.SetIMUMode;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Gains.Intake.Pivot;
import com.stuypulse.robot.subsystems.handoff.Handoff;
import com.stuypulse.robot.subsystems.handoff.Handoff.HandoffState;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import com.stuypulse.robot.subsystems.superstructure.hood.Hood;
import com.stuypulse.robot.subsystems.superstructure.shooter.Shooter;
import com.stuypulse.robot.subsystems.superstructure.turret.Turret;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.spindexer.Spindexer;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.vision.LimelightVision;
import com.stuypulse.robot.util.PathUtil.AutonConfig;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;
import com.stuypulse.stuylib.network.SmartBoolean;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;


public class RobotContainer {
    public interface EnabledSubsystems {
        SmartBoolean SWERVE = new SmartBoolean("Enabled Subsystems/Swerve Is Enabled", true);
        SmartBoolean TURRET = new SmartBoolean("Enabled Subsystems/Turret Is Enabled", true);
        SmartBoolean HANDOFF = new SmartBoolean("Enabled Subsystems/Handoff Is Enabled", true);
        SmartBoolean INTAKE = new SmartBoolean("Enabled Subsystems/Intake Is Enabled", true);
        SmartBoolean SPINDEXER = new SmartBoolean("Enabled Subsystems/Spindexer Is Enabled", true);
        SmartBoolean CLIMBER_HOPPER = new SmartBoolean("Enabled Subsystems/Climber-Hopper Is Enabled", false);
        SmartBoolean HOOD = new SmartBoolean("Enabled Subsystems/Hood Is Enabled", true);
        SmartBoolean SHOOTER = new SmartBoolean("Enabled Subsystems/Shooter Is Enabled", true);

        SmartBoolean BACK_LIMELIGHT = new SmartBoolean("Enabled Subsystems/Back Limelight Is Enabled", true);
        SmartBoolean LEFT_LIMELIGHT = new SmartBoolean("Enabled Subsystems/Left Limelight Is Enabled", true);
        SmartBoolean RIGHT_LIMELIGHT = new SmartBoolean("Enabled Subsystems/Right Limelight Is Enabled", true);
    }

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);

    // Subsystem
    // private final ClimberHopper climberHopper = ClimberHopper.getInstance();
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

        SmartDashboard.putData("Field", Field.FIELD2D);
        SmartDashboard.putData("Robot/Zero Pivot Encoder at Lower Limit (Deployed)", new ZeroPivotDeployed().ignoringDisable(true));
        SmartDashboard.putData("Robot/Zero Pivot Encoder at Upper Limit (Stowed)", new ZeroPivotStowed().ignoringDisable(true));
        SmartDashboard.putData("Robot/Zero Turret Encoders", new ZeroTurret().ignoringDisable(true));
        SmartDashboard.putData("Robot/Zero Hood Encoder", new ZeroHoodEncoderAtUpperHardstop().ignoringDisable(true));

        SmartDashboard.putData("Robot/Override Up", new ClimberOverrideUp());
        SmartDashboard.putData("Robot/Override Down", new ClimberOverrideDown());
        SmartDashboard.putData("Robot/Override Stop", new  ClimberOverrideStop());
       
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new SwerveDriveDrive(driver));
        // superstructure.setDefaultCommand(new SuperstructureDefaultCommand());
        // climberHopper.setDefaultCommand(new ClimberHopperDefaultCommand());
        // turret.setDefaultCommand(new TurretDefaultCommand());
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {
        // Scoring Routine
        driver.getBottomButton()
            .whileTrue(new SwerveXMode())
                .onTrue(new IntakeRunRollers())
                .whileTrue(new SuperstructureShoot()
                        .andThen(new WaitUntilCommand(superstructure::atTolerance))
                            .andThen(new HandoffRun())
                        .andThen(new WaitUntilCommand(handoff::atTolerance))
                            .andThen(new SpindexerRun()))
                .onFalse(new SpindexerStop()
                        .alongWith(new SuperstructureStow())
                        .alongWith(new HandoffStop()));

        // Interpolation-based Scoring
        driver.getTopButton()
            .whileTrue(new SwerveXMode())
                .onTrue(new IntakeRunRollers())
                .whileTrue(new SuperstructureInterpolation()
                        .andThen(new WaitUntilCommand(superstructure::atTolerance))
                            .andThen(new HandoffRun())
                        .andThen(new WaitUntilCommand(handoff::atTolerance))
                            .andThen(new SpindexerRun()))
                .onFalse(new SpindexerStop()
                        .alongWith(new SuperstructureStow())
                        .alongWith(new HandoffStop()));

        // Intake Stow
        driver.getLeftTriggerButton()
            .onTrue(new IntakeStow());

        // Intake Deploy
        driver.getRightTriggerButton()
        .onTrue(new IntakeDeploy());
        // .onTrue(new SuperstructureShoot()); // TODO: put shooter in shoot --> turn off sotm
        
        // Reset Heading
        driver.getDPadUp()
            .onTrue(new SwerveResetHeading())
            .onTrue(new ResetLimelightIMU())
            .onFalse(new SetIMUMode(0));   

        // Stop Rollers
        driver.getLeftBumper()
            .onTrue(new IntakeStopRollers());

        // driver.getRightBumper()
        //     .whileTrue(new IntakeOuttake())
        //     .onFalse(new IntakeRunRollers());

        // SOTM
        driver.getRightMenuButton()
            .onTrue(new IntakeRunRollers())
            .onTrue(new ConditionalCommand(
                new ParallelCommandGroup(
                    new SuperstructureInterpolation(),
                    new SpindexerStop(),
                    new HandoffStop()
                ),
                new SuperstructureSOTM().alongWith(new WaitUntilCommand(() -> superstructure.atTolerance()))
                    .andThen(new SpindexerRun()).alongWith(new HandoffRun()),
                () -> superstructure.getState() == SuperstructureState.SOTM
            ));

        // Ferrying In Place
        driver.getDPadRight()
            .whileTrue(new SwerveXMode())
            .whileTrue(new SuperstructureFerry().alongWith(new WaitUntilCommand(() -> superstructure.atTolerance()))
                .andThen(new HandoffRun())
                .andThen(new WaitUntilCommand(handoff::atTolerance))
                .alongWith(new SpindexerRun()))
            .onFalse(new SuperstructureFerry().alongWith(new SpindexerStop()).alongWith(new HandoffStop()));
        
        // FOTM
        // driver.getDPadRight()
        //     .onTrue(new IntakeRunRollers())
        //     .onTrue(new ConditionalCommand(
        //         new ParallelCommandGroup(
        //             new SuperstructureInterpolation(),
        //             new SpindexerStop(),
        //             new HandoffStop()
        //         ),
        //         new SuperstructureFOTM().alongWith(new WaitUntilCommand(() -> superstructure.atTolerance()))
        //             .andThen(new SpindexerRun()).alongWith(new HandoffRun()),
        //         () -> superstructure.getState() == SuperstructureState.FOTM
        //     ));

        // digestion by moving the pivot
        // driver.getDPadDown() 
        //     .whileTrue(new IntakeDigestion())
        //     .onFalse(new IntakeDeploy());

        // Test Turret
        // driver.getBottomButton()
        //     .whileTrue(new TurretShoot())
        //     .onFalse(new TurretIdle());

        // Scoring Routine
        // driver.getBottomButton()
        //         .whileTrue(new SuperstructureShoot())//.onlyIf(() -> !superstructure.isHoodUnderTrench()))
        //             // .alongWith(new SwerveDriveAlignTurretToHub())
        //             // .alongWith(new TurretShoot())
        //         .whileTrue(new SuperstructureShoot().onlyIf(() -> !swerve.isUnderTrench())
        //         .alongWith(new SwerveDriveAlignTurretToHub())
        //                 // .alongWith(new TurretShoot())
        //                 .andThen(new WaitUntilCommand(superstructure::atTolerance))
        //                 .andThen(new HandoffConditionalCommand().onlyIf(superstructure::atTolerance)
        //                         .alongWith(new WaitUntilCommand(handoff::atTolerance))
        //                         .andThen(new SpindexerConditionalCommand().onlyIf(() -> handoff.atTolerance() && superstructure.atTolerance()))))
        //         .onFalse(new SpindexerStop()
        //                 .alongWith(new SuperstructureStow())
        //                 .alongWith(new HandoffStop()));

        // driver.getRightBumper()
        //     .whileTrue(new SwerveWheelRadiusCharacterization());

        // driver.getRightButton()
        //     .whileTrue(new SuperstructureFerry().onlyIf(() -> !swerve.isUnderTrench())
        //             // .alongWith(new TurretShoot())
        //             .andThen(new WaitUntilCommand(superstructure::atTolerance))
        //             .andThen(new HandoffRun().onlyIf(superstructure::atTolerance)
        //                     .alongWith(new WaitUntilCommand(handoff::atTolerance))
        //                     .andThen(new SpindexerRun().onlyIf(() -> handoff.atTolerance() && superstructure.atTolerance()))))
        //     .onFalse(new SpindexerStop()
        //             .alongWith(new SuperstructureStow())
        //             .alongWith(new HandoffStop()));

//--------------------------------------------------------------------------------------------------------------------------\\

        // // Scoring In Place
        // driver.getTopButton()
        //     .whileTrue(new SwerveXMode())
        //     .whileTrue(new SuperstructureInterpolation().alongWith(new WaitUntilCommand(() -> superstructure.atTolerance()))
        //         .andThen(new SpindexerRun()).alongWith(new HandoffRun()))
        //     .onFalse(new SuperstructureInterpolation().alongWith(new SpindexerStop()).alongWith(new HandoffStop()));

        // // Manual Left Corner Scoring
        // driver.getLeftButton()
        //     .whileTrue(new SwerveXMode())
        //     .whileTrue(new SuperstructureLeftCorner().alongWith(new WaitUntilCommand(() -> superstructure.atTolerance()))
        //         .andThen(new SpindexerRun()).alongWith(new HandoffRun()))
        //     .onFalse(new SuperstructureInterpolation().alongWith(new SpindexerStop()).alongWith(new HandoffStop()));

        // // Manual Right Corner Scoring
        // driver.getRightButton()
        //     .whileTrue(new SwerveXMode())
        //     .whileTrue(new SuperstructureRightCorner().alongWith(new WaitUntilCommand(() -> superstructure.atTolerance()))
        //         .andThen(new SpindexerRun()).alongWith(new HandoffRun()))
        //     .onFalse(new SuperstructureInterpolation().alongWith(new SpindexerStop()).alongWith(new HandoffStop()));

        // // Manual KB Distance Scoring
        // driver.getBottomButton()
        //     .whileTrue(new SwerveXMode())
        //     .whileTrue(new SuperstructureKB().alongWith(new WaitUntilCommand(() -> superstructure.atTolerance()))
        //         .andThen(new SpindexerRun()).alongWith(new HandoffRun()))
        //     .onFalse(new SuperstructureInterpolation().alongWith(new SpindexerStop()).alongWith(new HandoffStop()));

        // // Reset Heading
        // driver.getDPadUp()
        //     .onTrue(new SwerveResetHeading())
        //     .onTrue(new ResetLimelightIMU())
        //     .onFalse(new SetIMUMode(0)); 


        // /** 
        // // Ferrying SOTM
        // driver.getLeftMenuButton()
        //     .onTrue(new SuperstructureSOTM().alongWith(new WaitUntilCommand(() -> superstructure.atTolerance()))
        //         .andThen(new SpindexerRun()).alongWith(new HandoffRun()))
        //     .onFalse(new SuperstructureSOTM().alongWith(new SpindexerStop()).alongWith(new HandoffStop()));
        // **/

        // // Scoring SOTM
        // driver.getRightMenuButton()
        //     .onTrue(new SuperstructureSOTM().alongWith(new WaitUntilCommand(() -> superstructure.atTolerance()))
        //         .andThen(new SpindexerRun()).alongWith(new HandoffRun()))
        //     .onFalse(new SuperstructureSOTM().alongWith(new SpindexerStop()).alongWith(new HandoffStop()));

        // // Swerve X Wheels
        // driver.getLeftBumper()
        //     .whileTrue(new SwerveXMode());

        // /** 
        // // Climb
        // driver.getRightBumper()
        //     .whileTrue(new ClimberUp().alongWith(new IntakeDeploy())
        //         .andThen(new SwerveClimbAlign()))
        //     .onFalse(new ClimberDown());
        // **/

        // // Stow Intake
        // driver.getLeftTriggerButton()
        //     .onTrue(new IntakeStow());

        // // Deploy Intake
        // driver.getRightTriggerButton()
        //     .onTrue(new IntakeDeploy());
    
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {

        autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());

        autonChooser.addOption("Wheel Radius", new SwerveWheelRadiusCharacterization());
        // TESTS
        // AutonConfig BOX_TEST = new AutonConfig("Box Test", BoxTest::new, 
        // "Box 1", "Box 2", "Box 3", "Box 4");
        // BOX_TEST.register(autonChooser);

        // BASE
        AutonConfig EIGHT_FUEL = new AutonConfig("Eight Fuel", EightFuel::new, 
        "");
        EIGHT_FUEL.register(autonChooser);

        // DEPOT
        AutonConfig DEPOT_AUTON = new AutonConfig("Depot Auton", DepotAuton::new, 
        "Top Bump To Depot", "Depot To Tower Left");
        DEPOT_AUTON.register(autonChooser);

        // ONE CYCLES
        AutonConfig TOP_ONE_CYCLE_POACH = new AutonConfig("Top One Cycle (Poach)", TopOneCyclePoach::new,  
        "Top Trench To NZ (P)", "Top NZ To Tower Left (P)");
        TOP_ONE_CYCLE_POACH.register(autonChooser);

        AutonConfig BOTTOM_ONE_CYCLE_POACH = new AutonConfig("Bottom One Cycle (Poach)", BottomOneCyclePoach::new,  
        "Bottom Trench To NZ (P)", "Bottom NZ To Tower Right (P)");
        BOTTOM_ONE_CYCLE_POACH.register(autonChooser);

        // TWO CYCLES
        AutonConfig TOP_TWO_CYCLE = new AutonConfig("Top Two Cycle", TopTwoCycle::new,  
        "Top Trench To NZ", "Top NZ To Score", "Top Score To NZ", "Top NZ To Tower Left");
        TOP_TWO_CYCLE.register(autonChooser);

        AutonConfig BOTTOM_TWO_CYCLE = new AutonConfig("Bottom Two Cycle", BottomTwoCycle::new,  
        "Bottom Trench To NZ", "Bottom NZ To Score", "Bottom Score To NZ", "Bottom NZ To Tower Right");
        BOTTOM_TWO_CYCLE.register(autonChooser);

        AutonConfig TOP_TWO_CYCLE_POACH = new AutonConfig("Top Two Cycle (Poach)", TopTwoCyclePoach::new,  
        "Top Trench To NZ (P)", "Top NZ To Score (P)", "Top Score To NZ", "Top NZ To Tower Left");  
        TOP_TWO_CYCLE_POACH.register(autonChooser);

        AutonConfig BOTTOM_TWO_CYCLE_POACH = new AutonConfig("Bottom Two Cycle (Poach)", BottomTwoCyclePoach::new,  
        "Bottom Trench To NZ (P)", "Bottom NZ To Score (P)", "Bottom Score To NZ", "Bottom NZ To Tower Right");
        BOTTOM_TWO_CYCLE_POACH.register(autonChooser);

        SmartDashboard.putData("Autonomous", autonChooser);

    }

    public void configureSysids() {

        autonChooser.addOption("SysID Module Translation Dynamic Forwards", swerve.sysIdDynamic(Direction.kForward));
        autonChooser.addOption("SysID Module Translation Dynamic Backwards", swerve.sysIdDynamic(Direction.kReverse));
        autonChooser.addOption("SysID Module Translation Quasi Forwards", swerve.sysIdQuasistatic(Direction.kForward));
        autonChooser.addOption("SysID Module Translation Quasi Backwards", swerve.sysIdQuasistatic(Direction.kReverse)); 

        autonChooser.addOption("SysID Rotation Translation Dynamic Forwards", swerve.sysidRotationDynamic(Direction.kForward));
        autonChooser.addOption("SysID Rotation Translation Dynamic Backwards", swerve.sysidRotationDynamic(Direction.kReverse));
        autonChooser.addOption("SysID Rotation Translation Quasi Forwards", swerve.sysidRotationQuasiStatic(Direction.kForward));
        autonChooser.addOption("SysID Rotation Translation Quasi Backwards", swerve.sysidRotationQuasiStatic(Direction.kReverse)); 
        

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

        SysIdRoutine spindexerSysId = spindexer.getSysIdRoutine();
        autonChooser.addOption("SysID Spindexer Dynamic Forwards", spindexerSysId.dynamic(Direction.kForward));
        autonChooser.addOption("SysID Spindexer Dynamic Backwards", spindexerSysId.dynamic(Direction.kReverse));
        autonChooser.addOption("SysID Spindexer Quasi Forwards", spindexerSysId.quasistatic(Direction.kForward));
        autonChooser.addOption("SysID Spindexer Quasi Backwards", spindexerSysId.quasistatic(Direction.kReverse));

        // Wheel Radius Characterization
        autonChooser.addOption("Wheel Characterization", new SwerveWheelRadiusCharacterization());

        SysIdRoutine handoffSysId = handoff.getSysIdRoutine();
        autonChooser.addOption("SysID Handoff Dynamic Forward", handoffSysId.dynamic(Direction.kForward));
        autonChooser.addOption("SysID Handoff Dynamic Backwards", handoffSysId.dynamic(Direction.kReverse));
        autonChooser.addOption("SysID Handoff Quasi Forwards", handoffSysId.quasistatic(Direction.kForward));
        autonChooser.addOption("SysID Handoff Quasi Backwards", handoffSysId.quasistatic(Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}