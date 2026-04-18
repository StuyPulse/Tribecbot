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
import com.stuypulse.robot.util.superstructure.InterpolationCalculator;
import com.stuypulse.robot.util.superstructure.SOTMCalculator;
import com.stuypulse.robot.util.superstructure.VisualizerHood;
import com.stuypulse.stuylib.input.Gamepad;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
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
        MANUAL_OVERRIDE,
        KB,
        LEFT_CORNER,
        RIGHT_CORNER,
        INTERPOLATION,
        SOTM,
        FOTM,
        ANALOG,
        HOMING_UPPER,
        HOMING_LOWER,
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
            case FERRY -> InterpolationCalculator.getInterpolatedFerryAngle();
            case MANUAL_OVERRIDE -> Rotation2d.fromDegrees(Settings.Superstructure.Hood.Angles.MANUAL_OVERRIDE.get());
            case KB -> Settings.Superstructure.Hood.Angles.KB;
            case LEFT_CORNER -> Settings.Superstructure.Hood.Angles.LEFT_CORNER;
            case RIGHT_CORNER -> Settings.Superstructure.Hood.Angles.RIGHT_CORNER;
            case INTERPOLATION -> InterpolationCalculator.getInterpolatedShotAngle();
            case HOMING_UPPER -> new Rotation2d(); //should just apply a voltage, not an angle!
            case HOMING_LOWER -> new Rotation2d();
            case SOTM -> SOTMCalculator.calculateHoodAngleSOTM();
            case FOTM -> SOTMCalculator.calculateHoodAngleFOTM();
            case ANALOG -> hoodAnalogToOutput();
            case IDLE -> getAngle();
        };
    }

    public boolean atTolerance() {
        double error = getAngle().minus(getTargetAngle()).getRotations();
        if (Robot.isReal()) {
            if (state == HoodState.SOTM || state == HoodState.FOTM) {
                return Math.abs(error) < Settings.Superstructure.HOOD_SOTM_TOLERANCE.getRotations();
            } else {
                return Math.abs(error) < Settings.Superstructure.HOOD_TOLERANCE.getRotations();
            }
        } else {
            return Math.abs(error) < Settings.Superstructure.HOOD_TOLERANCE.getRotations() + (5 / 360.0);
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

    public abstract void seedHoodAtUpperHardStop();
    public abstract void seedHoodAtLowerHardStop();
    public abstract void zeroHoodEncodersAfterSeed();
    
    public abstract double getCurrentDraw();


    public void periodicAfterScheduler() {
        DogLog.log("Superstructure/Hood/State", state.name());

        DogLog.log("Superstructure/Hood/Target Angle (deg)", getTargetAngle().getDegrees());
        DogLog.log("Superstructure/Hood/Current Angle (deg)", getAngle().getDegrees());

        if (Settings.DEBUG_MODE.get()) {
            if (EnabledSubsystems.HOOD.get()) {
                    VisualizerHood.getInstance().update(getAngle(), atTolerance());
            } else {
                // VisualizerHood.getInstance().update(new Rotation2d(), false);
            }
        }
    }
}