/************************ PROJECT ALPHA *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.hoodedshooter;

import com.stuypulse.robot.subsystems.hoodedshooter.hood.Hood;
import com.stuypulse.robot.subsystems.hoodedshooter.hood.Hood.HoodState;
import com.stuypulse.robot.subsystems.hoodedshooter.shooter.Shooter;
import com.stuypulse.robot.subsystems.hoodedshooter.shooter.Shooter.ShooterState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodedShooter extends SubsystemBase {

    private static final HoodedShooter instance;

    static {
        instance = new HoodedShooter();
    }
    
    public static HoodedShooter getInstance(){
        return instance;
    }

    private HoodedShooterState state;

    private final Hood hood;
    private final Shooter shooter;

    public HoodedShooter() {
        state = HoodedShooterState.INTERPOLATION;
        hood = Hood.getInstance();
        shooter = Shooter.getInstance();
    }
    
    public enum HoodedShooterState {
        STOW(HoodState.STOW, ShooterState.SHOOT),
        SHOOT(HoodState.SHOOT, ShooterState.SHOOT),
        FERRY(HoodState.FERRY, ShooterState.FERRY),
        REVERSE(HoodState.SHOOT, ShooterState.REVERSE),
        HUB(HoodState.HUB, ShooterState.HUB),
        LEFT_CORNER(HoodState.LEFT_CORNER, ShooterState.LEFT_CORNER),
        RIGHT_CORNER(HoodState.RIGHT_CORNER, ShooterState.RIGHT_CORNER),
        INTERPOLATION(HoodState.INTERPOLATION, ShooterState.INTERPOLATION);

        private HoodState hoodState;
        private ShooterState shooterState;

        private HoodedShooterState(HoodState hoodState, ShooterState shooterState) {
            this.hoodState = hoodState;
            this.shooterState = shooterState;
        }

        public HoodState getHoodState() {
            return hoodState;
        }

        public ShooterState getShooterState() {
            return shooterState;
        }
    }

    public void setState(HoodedShooterState state){
        this.state = state;
        hood.setState(state.getHoodState());
        shooter.setState(state.getShooterState());
    }

    public HoodedShooterState getState(){
        return state;
    }

    public boolean bothAtTolerance() {
        return isShooterAtTolerance() && isHoodAtTolerance();
    }

    public boolean isShooterAtTolerance() {
        return shooter.atTolerance();
    }

    public boolean isHoodAtTolerance() {
        return hood.atTolerance();
    }

    public double getTargetRPM() {
        return shooter.getTargetRPM();
    }

    public Rotation2d getTargetAngle() {
        return hood.getTargetAngle();
    }

    public double getShooterRPM() {
        return shooter.getShooterRPM();
    }

    public Rotation2d getHoodAngle() {
        return hood.getHoodAngle();
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("HoodedShooter/State", state.name());
        SmartDashboard.putString("States/HoodedShooter", state.name());

        SmartDashboard.putNumber("HoodedShooter/Target RPM", shooter.getTargetRPM());
        SmartDashboard.putNumber("HoodedShooter/Target Angle", hood.getTargetAngle().getDegrees());

        SmartDashboard.putNumber("HoodedShooter/Current RPM", getShooterRPM());
        SmartDashboard.putNumber("HoodedShooter/Current Angle", getHoodAngle().getDegrees());

        SmartDashboard.putNumber("HoodedShooter/Angle Error (Deg)", hood.getTargetAngle().getDegrees() - getHoodAngle().getDegrees());

        SmartDashboard.putBoolean("HoodedShooter/Shooter At Tolerance?", isShooterAtTolerance());
        SmartDashboard.putBoolean("HoodedShooter/Hood At Tolerance?", isHoodAtTolerance());

        SmartDashboard.putNumber("InterpolationTesting/Hood Angle", getHoodAngle().getDegrees());
        SmartDashboard.putNumber("InterpolationTesting/Shooter RPM", getShooterRPM());
    }
}