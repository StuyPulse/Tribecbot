/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.hoodedshooter.hood;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.hoodedshooter.HoodAngleCalculator;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public abstract class Hood extends SubsystemBase{
    private static final Hood instance;
    
    private HoodState state;

    private Rotation2d driverInput;

    static {
        instance = new HoodImpl();
    }
    
    public static Hood getInstance(){
        return instance;
    }
    
    public enum HoodState {
        STOW,
        FERRY,
        SHOOT,
        KB,
        LEFT_CORNER,
        RIGHT_CORNER,
        INTERPOLATION,
        ANALOG,
        IDLE;
    }

    public Hood() {
        state = HoodState.STOW;
    }

    public HoodState getState(){
        return state;
    }

    public void setState(HoodState state){
        this.state = state;
    }

    public Rotation2d getTargetAngle() {
        return switch(state) {
            case STOW -> Settings.HoodedShooter.Hood.Angles.MIN_ANGLE;
            case FERRY -> Rotation2d.fromDegrees(30);
            case SHOOT -> Rotation2d.fromDegrees(Settings.HoodedShooter.Hood.Angles.SHOOT_ANGLE.get());
            case KB -> Settings.HoodedShooter.Hood.Angles.KB_ANGLE;
            case LEFT_CORNER -> Settings.HoodedShooter.Hood.Angles.LEFT_CORNER_ANGLE;
            case RIGHT_CORNER -> Settings.HoodedShooter.Hood.Angles.RIGHT_CORNER_ANGLE;
            case INTERPOLATION -> HoodAngleCalculator.interpolateHoodAngle().get();
            case ANALOG -> hoodAnalogToOutput();
            case IDLE -> getHoodAngle();
        };
    }

    public boolean atTolerance() {
        double error = getHoodAngle().minus(getTargetAngle()).getRotations();
        return Math.abs(error) < Settings.HoodedShooter.HOOD_TOLERANCE.getRotations();
    }

    public abstract Rotation2d getHoodAngle();

    public void hoodAnalogToInput(Gamepad gamepad) {
        double hoodMin = Settings.HoodedShooter.Hood.Angles.MIN_ANGLE.getDegrees();
        double hoodMax = Settings.HoodedShooter.Hood.Angles.MAX_ANGLE.getDegrees();

        this.driverInput = Rotation2d.fromDegrees(hoodMin + (gamepad.getLeftX() + 1.0) * ((hoodMax - hoodMin) / 2)); 
    }

    public Rotation2d hoodAnalogToOutput() {
        return this.driverInput;
    }

    public boolean isHoodUnderTrench() {
        Pose2d pose = CommandSwerveDrivetrain.getInstance().getTurretPose();

        boolean isBetweenRightTrenchesY = Field.NearRightTrench.rightEdge.getY() < pose.getY() && Field.NearRightTrench.leftEdge.getY() > pose.getY();

        boolean isBetweenLeftTrenchesY = Field.NearLeftTrench.rightEdge.getY() < pose.getY() && Field.NearLeftTrench.leftEdge.getY() > pose.getY();

        boolean isCloseToAllianceSideTrenchX = Math.abs(pose.getX() - Field.NearRightTrench.rightEdge.getX()) < Field.trenchHoodTolerance;

        boolean isCloseToNeutralSideTrenchX = Math.abs(pose.getX() - Field.FarRightTrench.rightEdge.getX()) < Field.trenchHoodTolerance;

        boolean isUnderTrench = (isBetweenRightTrenchesY || isBetweenLeftTrenchesY) && (isCloseToAllianceSideTrenchX || isCloseToNeutralSideTrenchX);
        
        return isUnderTrench;
    }
    
    public abstract boolean isStalling();
    public abstract SysIdRoutine getHoodSysIdRoutine();
    public abstract void zeroHoodEncoderAtLowerHardstop();
    public abstract void seedHood();

    @Override
    public void periodic() {
        SmartDashboard.putString("HoodedShooter/Hood/State", state.name());
        SmartDashboard.putString("States/Hood", state.name());

        SmartDashboard.putNumber("HoodedShooter/Hood/Target Angle", getTargetAngle().getDegrees());
        SmartDashboard.putNumber("HoodedShooter/Hood/Current Angle", getHoodAngle().getDegrees());

        //SmartDashboard.putNumber("HoodedShooter/Hood/Analog Target Angle", hoodAnalogToOutput().getDegrees());

        SmartDashboard.putBoolean("HoodedShooter/Hood/Under Trench", isHoodUnderTrench());
    }
}