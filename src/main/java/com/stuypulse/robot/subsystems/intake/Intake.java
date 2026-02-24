/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.Optional;

public abstract class Intake extends SubsystemBase {
    private static final Intake instance;
    private IntakeState state;

    static {
        instance = new IntakeImpl();
    }

    public static Intake getInstance() {
        return instance;
    }

    public Intake() {
        state = IntakeState.STOP;
    }

    public enum IntakeState {
        PIVOTOUT(Settings.Intake.PIVOT_INTAKE_OUTAKE_ANGLE, 0.0),
        PIVOTIN(Settings.Intake.PIVOT_STOW_ANGLE, 0.0),

        INTAKE(Settings.Intake.PIVOT_INTAKE_OUTAKE_ANGLE, 1.0),
        OUTAKE(Settings.Intake.PIVOT_INTAKE_OUTAKE_ANGLE, -1.0),
        STOP(Settings.Intake.PIVOT_INTAKE_OUTAKE_ANGLE, 0.0);

        private double targetDutyCycle;
        private Rotation2d targetAngle;

        private IntakeState(Rotation2d targetAngle, double targetDutyCycle) {
            this.targetAngle = targetAngle;
            this.targetDutyCycle = targetDutyCycle;
        }

        public Rotation2d getTargetAngle() {
            return targetAngle;
        }

        public double getTargetDutyCycle() {
            return targetDutyCycle;
        }
    }

    public IntakeState getState() {
        return state;
    }

    public void setState(IntakeState state) {
        this.state = state;
    }

    public abstract boolean pivotAtTolerance();
    public abstract boolean rollerStopped();
    public abstract Rotation2d getPivotAngle();
    public abstract SysIdRoutine getPivotSysIdRoutine();
    public abstract void setPivotVoltageOverride(Optional<Double> voltage);
    
    @Override
    public void periodic() {
        SmartDashboard.putString("Intake/State", getState().toString());
        SmartDashboard.putString("Intake/State", getState().toString());

        SmartDashboard.putNumber("Intake/Current Angle (deg)", getPivotAngle().getDegrees());
        SmartDashboard.putNumber("Intake/Target Angle (deg)", getState().getTargetAngle().getDegrees());

        SmartDashboard.putNumber("Intake/Target Duty Cycle", getState().getTargetDutyCycle());

        SmartDashboard.putBoolean("Intake/Pivot At Tolerance?", pivotAtTolerance());
    }

}
