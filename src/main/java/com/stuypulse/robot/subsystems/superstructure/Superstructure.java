/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.superstructure;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;
import com.stuypulse.robot.subsystems.hood.Hood;
import com.stuypulse.robot.subsystems.hood.Hood.HoodState;
import com.stuypulse.robot.subsystems.turret.Turret;
import com.stuypulse.robot.subsystems.turret.Turret.TurretState;

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
        STOW(HoodState.STOW, ShooterState.SHOOT, TurretState.SHOOT),
        SHOOT(HoodState.SHOOT, ShooterState.SHOOT, TurretState.SHOOT),
        FERRY(HoodState.FERRY, ShooterState.FERRY, TurretState.FERRY),
        REVERSE(HoodState.SHOOT, ShooterState.REVERSE, TurretState.SHOOT),
        KB(HoodState.KB, ShooterState.KB, TurretState.SHOOT),
        LEFT_CORNER(HoodState.LEFT_CORNER, ShooterState.LEFT_CORNER, TurretState.SHOOT),
        RIGHT_CORNER(HoodState.RIGHT_CORNER, ShooterState.RIGHT_CORNER, TurretState.SHOOT),
        INTERPOLATION(HoodState.INTERPOLATION, ShooterState.INTERPOLATION, TurretState.SHOOT);

        private HoodState hoodState;
        private ShooterState shooterState;
        private TurretState  turretState;

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

    public boolean isHoodUnderTrench() {
        return hood.isHoodUnderTrench();
    }

    public boolean isShooterAtTolerance() {
        return shooter.atTolerance();
    }

    public boolean isHoodAtTolerance() {
        return hood.atTolerance();
    }

    public boolean isTurretAtTolerance(){
        return turret.atTargetAngle();
    }

    public double getTargetRPM() {
        return shooter.getTargetRPM();
    }

    public Rotation2d getTargetYaw() {
        return hood.getTargetAngle();
    }

    public Rotation2d getTargetPitch(){
        return turret.getTargetAngle();
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

    @Override
    public void periodic() {
        SmartDashboard.putString("SuperStructure/State", state.name());
        SmartDashboard.putString("States/SuperStructure", state.name());

        SmartDashboard.putNumber("SuperStructure/Target RPM", getTargetRPM());
        SmartDashboard.putNumber("SuperStructure/Target Yaw", getTargetYaw().getDegrees());
        SmartDashboard.putNumber("SuperStructure/Target Pitch", getTargetPitch().getDegrees());

        SmartDashboard.putNumber("SuperStructure/Current RPM", getShooterRPM());
        SmartDashboard.putNumber("SuperStructure/Current Yaw", getTurretAngle().getDegrees());
        SmartDashboard.putNumber("SuperStructure/Current Pitch", getHoodAngle().getDegrees());

        SmartDashboard.putNumber("SuperStructure/Hood Angle Error (Deg)", getTargetPitch().getDegrees() - getHoodAngle().getDegrees());

        SmartDashboard.putBoolean("SuperStructure/Shooter At Tolerance?", isShooterAtTolerance());
        SmartDashboard.putBoolean("SuperStructure/Hood At Tolerance?", isHoodAtTolerance());
        SmartDashboard.putBoolean("SuperStructure/Turret At Tolerant?", isHoodAtTolerance());

        SmartDashboard.putNumber("InterpolationTesting/Hood Angle", getHoodAngle().getDegrees());
        SmartDashboard.putNumber("InterpolationTesting/Shooter RPM", getShooterRPM());
    }
}