/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.superstructure;

import com.stuypulse.robot.subsystems.superstructure.hood.Hood;
import com.stuypulse.robot.subsystems.superstructure.hood.Hood.HoodState;
import com.stuypulse.robot.subsystems.superstructure.shooter.Shooter;
import com.stuypulse.robot.subsystems.superstructure.shooter.Shooter.ShooterState;
import com.stuypulse.robot.subsystems.superstructure.turret.Turret;
import com.stuypulse.robot.subsystems.superstructure.turret.Turret.TurretState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.superstructure.SOTMCalculator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {

    private static final Superstructure instance;

    static {
        instance = new Superstructure();
    }
    
    public static Superstructure getInstance(){
        return instance;
    }

    private SuperstructureState state;

    private final Hood hood;
    private final Shooter shooter;
    private final Turret turret;

    public Superstructure() {
        state = SuperstructureState.INTERPOLATION;
        hood = Hood.getInstance();
        shooter = Shooter.getInstance();
        turret = Turret.getInstance();
    }
    
    public enum SuperstructureState {
        STOW(HoodState.STOW, ShooterState.INTERPOLATION, TurretState.SHOOT),
        SHOOT(HoodState.SHOOT, ShooterState.SHOOT, TurretState.SHOOT),
        FERRY(HoodState.FERRY, ShooterState.FERRY, TurretState.FERRY),
        FOTM(HoodState.FOTM, ShooterState.FOTM, TurretState.FOTM),
        REVERSE(HoodState.SHOOT, ShooterState.REVERSE, TurretState.SHOOT),
        KB(HoodState.KB, ShooterState.KB, TurretState.KB),
        LEFT_CORNER(HoodState.LEFT_CORNER, ShooterState.LEFT_CORNER, TurretState.LEFT_CORNER),
        RIGHT_CORNER(HoodState.RIGHT_CORNER, ShooterState.RIGHT_CORNER, TurretState.RIGHT_CORNER),
        INTERPOLATION(HoodState.INTERPOLATION, ShooterState.INTERPOLATION, TurretState.SHOOT),
        AUTO_INTERPOLATION(HoodState.STOW, ShooterState.INTERPOLATION, TurretState.SHOOT),
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

    @Override
    public void periodic() {
        SuperstructureState state = getState();
        if (state == SuperstructureState.SOTM) {
            SOTMCalculator.updateSOTMSolution();
        } else if (state == SuperstructureState.FOTM){
            SOTMCalculator.updateFOTMSolution();
        }
        
        if (CommandSwerveDrivetrain.getInstance().isOutsideAllianceZone() && state == SuperstructureState.SOTM) {
            setState(SuperstructureState.FERRY);
        }

        SmartDashboard.putString("Superstructure/State", state.name());
        SmartDashboard.putString("States/SuperStructure", state.name());

        SmartDashboard.putBoolean("Superstructure/Shooter At Tolerance?", isShooterAtTolerance());
        SmartDashboard.putBoolean("Superstructure/Hood At Tolerance?", isHoodAtTolerance());
        SmartDashboard.putBoolean("Superstructure/Turret At Tolerance?", isTurretAtTolerance());
    }
}