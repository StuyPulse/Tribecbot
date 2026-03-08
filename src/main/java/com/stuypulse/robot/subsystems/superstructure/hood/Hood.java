/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.superstructure.hood;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.superstructure.SOTMCalculator;
import com.stuypulse.robot.util.superstructure.VisualizerHood;
import com.stuypulse.robot.util.superstructure.InterpolationCalculator;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public abstract class Hood extends SubsystemBase{
    private static final Hood instance;
    
    private HoodState state;

    private Rotation2d driverInput;

    static {
        if (Robot.isReal()) {
            instance = new HoodImpl();
        } else {
            instance = new HoodSim();
        }
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
        SOTM,
        FOTM,
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
        if (CommandSwerveDrivetrain.getInstance().isUnderTrench()) {
            return Settings.Superstructure.Hood.Angles.STOW;
        }

        return switch(state) {
            case STOW -> Settings.Superstructure.Hood.Angles.STOW;
            case FERRY -> Rotation2d.fromDegrees(30);
            case SHOOT -> Rotation2d.fromDegrees(Settings.Superstructure.Hood.Angles.SHOOT.get());
            case KB -> Settings.Superstructure.Hood.Angles.KB;
            case LEFT_CORNER -> Settings.Superstructure.Hood.Angles.LEFT_CORNER;
            case RIGHT_CORNER -> Settings.Superstructure.Hood.Angles.RIGHT_CORNER;
            case INTERPOLATION -> InterpolationCalculator.interpolateShotInfo().targetHoodAngle();
            case SOTM -> SOTMCalculator.calculateHoodAngleSOTM();
            case FOTM -> SOTMCalculator.calculateHoodAngleFOTM();
            case ANALOG -> hoodAnalogToOutput();
            case IDLE -> getAngle();
        };
    }

    public boolean atTolerance() {
        double error = getAngle().minus(getTargetAngle()).getRotations();
        if (Robot.isReal()) {
            return Math.abs(error) < Settings.Superstructure.HOOD_TOLERANCE.getRotations();
        } else {
            return Math.abs(error) < Settings.Superstructure.HOOD_TOLERANCE.getRotations() + (0.5 / 360.0);
        }
    }

    public abstract Rotation2d getAngle();

    public void hoodAnalogToInput(Gamepad gamepad) {
        double hoodMin = Settings.Superstructure.Hood.Angles.MIN.getDegrees();
        double hoodMax = Settings.Superstructure.Hood.Angles.MAX.getDegrees();

        this.driverInput = Rotation2d.fromDegrees(hoodMin + (gamepad.getLeftX() + 1.0) * ((hoodMax - hoodMin) / 2)); 
    }

    public Rotation2d hoodAnalogToOutput() {
        return this.driverInput;
    }
    
    public abstract boolean isStalling();

    public abstract SysIdRoutine getHoodSysIdRoutine();

    public abstract void zeroHoodEncoderAtUpperHardstop();
    public abstract void seedHood();

    @Override
    public void periodic() {
        SmartDashboard.putString("Superstructure/Hood/State", state.name());
        SmartDashboard.putString("States/Hood", state.name());

        SmartDashboard.putNumber("Superstructure/Hood/Target Angle", getTargetAngle().getDegrees());
        SmartDashboard.putNumber("Superstructure/Hood/Current Angle", getAngle().getDegrees());

        if (Settings.DEBUG_MODE) {
            if (EnabledSubsystems.HOOD.get()) {
                    VisualizerHood.getInstance().update(getAngle(), atTolerance());
            } else {
                // VisualizerHood.getInstance().update(new Rotation2d(), false);
            }
        }
    }
}