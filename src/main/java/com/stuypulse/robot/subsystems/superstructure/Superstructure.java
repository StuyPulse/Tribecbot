/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.superstructure;

import java.util.Optional;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.RobotMode;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.handoff.Handoff;
import com.stuypulse.robot.subsystems.handoff.Handoff.HandoffState;
import com.stuypulse.robot.subsystems.spindexer.Spindexer;
import com.stuypulse.robot.subsystems.spindexer.Spindexer.SpindexerState;
import com.stuypulse.robot.subsystems.superstructure.hood.Hood;
import com.stuypulse.robot.subsystems.superstructure.hood.Hood.HoodState;
import com.stuypulse.robot.subsystems.superstructure.shooter.Shooter;
import com.stuypulse.robot.subsystems.superstructure.shooter.Shooter.ShooterState;
import com.stuypulse.robot.subsystems.superstructure.turret.Turret;
import com.stuypulse.robot.subsystems.superstructure.turret.Turret.TurretState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {

    private static final Superstructure instance;

    private final Timer sotmStoppedTimer;
    private final Timer fotmStoppedTimer;

    static {
        instance = new Superstructure();
    }
    
    public static Superstructure getInstance(){
        return instance;
    }

    private SuperstructureState state;

    private Optional<Boolean> shouldStop;

    private final Hood hood;
    private final Shooter shooter;
    private final Turret turret;

    // private final BStream readyToShoot;

    public Superstructure() {
        state = SuperstructureState.INTERPOLATION;
        hood = Hood.getInstance();
        shooter = Shooter.getInstance();
        turret = Turret.getInstance();

        // readyToShoot = BStream.create(this::atTolerance)
        //     .filtered(new BDebounce.Both(0.05));

        sotmStoppedTimer = new Timer();
        sotmStoppedTimer.restart();
        sotmStoppedTimer.stop();
            

        fotmStoppedTimer = new Timer();
        fotmStoppedTimer.restart();
        fotmStoppedTimer.stop();

        this.shouldStop = Optional.empty();
    }
    
    public enum SuperstructureState {
        STOW(HoodState.STOW, ShooterState.INTERPOLATION, TurretState.SCORE),
        MANUAL_OVERRIDE(HoodState.MANUAL_OVERRIDE, ShooterState.MANUAL_OVERRIDE, TurretState.SCORE),
        FERRY(HoodState.FERRY, ShooterState.FERRY, TurretState.FERRY),
        FOTM(HoodState.FOTM, ShooterState.FOTM, TurretState.FOTM),
        REVERSE(HoodState.MANUAL_OVERRIDE, ShooterState.REVERSE, TurretState.SCORE),
        KB(HoodState.KB, ShooterState.KB, TurretState.KB),
        LEFT_CORNER(HoodState.LEFT_CORNER, ShooterState.LEFT_CORNER, TurretState.LEFT_CORNER),
        RIGHT_CORNER(HoodState.RIGHT_CORNER, ShooterState.RIGHT_CORNER, TurretState.RIGHT_CORNER),
        INTERPOLATION(HoodState.INTERPOLATION, ShooterState.INTERPOLATION, TurretState.SCORE),
        AUTO_INTERPOLATION(HoodState.STOW, ShooterState.INTERPOLATION, TurretState.SCORE),
        AUTO_INTERPOLATION_SOTM(HoodState.STOW, ShooterState.SOTM, TurretState.SOTM),
        SOTM(HoodState.SOTM, ShooterState.SOTM, TurretState.SOTM);

        private HoodState hoodState;
        private ShooterState shooterState;
        private TurretState turretState;

        private SuperstructureState(HoodState hoodState, ShooterState shooterState, TurretState TurretState) {
            this.hoodState = hoodState;
            this.shooterState = shooterState;
            this.turretState = TurretState;
        }

        public HoodState getHoodState() {
            return hoodState;
        }

        public ShooterState getShooterState() {
            return shooterState;
        }

        public TurretState getTurretState(){
            return turretState;
        }
    }

    public void setState(SuperstructureState state){
        this.state = state;
        hood.setState(state.getHoodState());
        shooter.setState(state.getShooterState());
        turret.setState(state.getTurretState());
    }

    public SuperstructureState getState(){
        return state;
    }

    public boolean isReadyToShoot() {
        return turret.turretReadyToShoot() && shooter.shooterReadyToShoot() && hood.hoodReadyToShoot();
    }

    public boolean atTolerance() {
        return isShooterAtTolerance() && isHoodAtTolerance() && isTurretAtTolerance();
    }
    
    public boolean isShooterAtTolerance() {
        return shooter.atTolerance();
    }

    public boolean isHoodAtTolerance() {
        return hood.atTolerance();
    }

    public boolean isTurretAtTolerance(){
        return turret.atTolerance();
    }

    public boolean isTurretLaggingFOTM() {
        double error = Math.abs(turret.getAngle().minus(turret.getTargetAngle()).getRotations());
        return error >= Settings.Superstructure.Turret.GAIN_SWITCHING_THRESHOLD_START.getRotations() && getState() == SuperstructureState.FOTM;
    }

    public double getTargetRPM() {
        return shooter.getTargetRPM();
    }

    public double getShooterRPM() {
        return shooter.getRPM();
    }

    public Rotation2d getHoodAngle() {
        return hood.getAngle();
    }

    public Rotation2d getTurretAngle(){
        return turret.getAngle();
    }

    public boolean isTurretWrapping() {
        return turret.isWrapping();
    }

    public double getCurrentDraw() {
        return turret.getCurrentDraw() + shooter.getCurrentDraw() + hood.getCurrentDraw();
    }
    
    public boolean shouldStop() {
        if (!shouldStop.isEmpty()) {
            return shouldStop.get();
        }
        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

        boolean isSpindexerStopState = Spindexer.getInstance().getState() == SpindexerState.STOP;
        boolean isHandOffStopState = Handoff.getInstance().getState() == HandoffState.STOP;

        boolean isTurretWrapping = isTurretWrapping();
        boolean isBehindHubWhileFerrying = getState() == SuperstructureState.FOTM
                && swerve.isBehindHub();
        boolean isOutsideAllianceZone = 
            CommandSwerveDrivetrain.getInstance().isOutsideAllianceZone() && 
            getState() != SuperstructureState.FOTM;
        boolean isUnderTrench = CommandSwerveDrivetrain.getInstance().isUnderTrench() 
            && getState() != SuperstructureState.FOTM;
        boolean inManualState =       
            getState() == SuperstructureState.LEFT_CORNER &&
            getState() == SuperstructureState.RIGHT_CORNER &&
            getState() == SuperstructureState.KB;
        boolean isBehindTower = swerve.isBehindTower() && getState() == SuperstructureState.SOTM;
        boolean isBtwnOppHubAndWall = swerve.isBtwnOppHubAndWall() && getState() == SuperstructureState.FOTM;

        boolean turretLaggingSOTM = !isTurretAtTolerance() && getState() == SuperstructureState.SOTM;
        boolean turretLaggingFOTM = isTurretLaggingFOTM();

        DogLog.log("Spindexer/Should Stop/Is Behind Tower?", isBehindTower);
        DogLog.log("Spindexer/Should Stop/Is Behind Hub While Ferrying?", isBehindHubWhileFerrying);
        DogLog.log("Spindexer/Should Stop/Is behind Opponent's Hub While Ferrying?", isBtwnOppHubAndWall);
        DogLog.log("Spindexer/Should Stop/Is Turret Wrapping?", isTurretWrapping);
        DogLog.log("Spindexer/Should Stop/Is Outside Alliance Zone?", isOutsideAllianceZone);
        DogLog.log("Spindexer/Should Stop/Is Under Trench?", isUnderTrench);
        DogLog.log("Spindexer/Should Stop/Turret Lagging SOTM", turretLaggingSOTM);
        DogLog.log("Spindexer/Should Stop/Turret Lagging FOTM", turretLaggingFOTM);
        DogLog.log("Spindexer/Should Stop/In Manual State", inManualState);
        
        boolean shouldStop = isSpindexerStopState || 
            isHandOffStopState ||
            (isBehindHubWhileFerrying && !inManualState) || 
            isBtwnOppHubAndWall ||
            turretLaggingSOTM || 
            turretLaggingFOTM ||
            (isOutsideAllianceZone  && !inManualState) || 
            (isUnderTrench && !inManualState) ||
            isBehindTower;

        this.shouldStop = Optional.of(shouldStop);

        return shouldStop;
    }

    public void clearMemoized() {
        this.shouldStop = Optional.empty();
    }

    public boolean isHopperEmpty() {
        return !shooter.isShooting();
    }

    public void periodicAfterScheduler() {
        SuperstructureState state = getState();
        
        if (getState() == SuperstructureState.SOTM && shouldStop() && DriverStation.isEnabled()) {
            sotmStoppedTimer.start();
        }
        else if (getState() == SuperstructureState.FOTM && shouldStop() && DriverStation.isEnabled()) {
            fotmStoppedTimer.start();
        }
        
        if (getState() != SuperstructureState.SOTM) sotmStoppedTimer.stop();
        if (getState() != SuperstructureState.FOTM) fotmStoppedTimer.stop();
        
        if (!shouldStop() || DriverStation.isDisabled()) {
            sotmStoppedTimer.stop();
            fotmStoppedTimer.stop();
        }

        if (CommandSwerveDrivetrain.getInstance().isOutsideAllianceZone() && state == SuperstructureState.SOTM &&
            Robot.getMode() != RobotMode.AUTON) { // allows us to start SOTM earlier in auto, but currently not desired in teleop
            setState(SuperstructureState.STOW);
            Spindexer.getInstance().setState(SpindexerState.STOP);
            Handoff.getInstance().setState(HandoffState.STOP);
        }

        DogLog.log("Superstructure/State", state.name());

        DogLog.log("Superstructure/SOTM Stopped Timer", sotmStoppedTimer.get());
        DogLog.log("Superstructure/FOTM Stopped Timer", fotmStoppedTimer.get());

        DogLog.log("Superstructure/Shooter At Tolerance?", isShooterAtTolerance());
        DogLog.log("Superstructure/Hood At Tolerance?", isHoodAtTolerance());
        DogLog.log("Superstructure/Turret At Tolerance?", isTurretAtTolerance());

        DogLog.log("Superstructure/Is Hopper Empty?", isHopperEmpty());
        DogLog.log("Superstructure/Is Ready To Shoot?", isReadyToShoot());

        DogLog.log("Superstructure/Should Stop?", shouldStop());
    }
}