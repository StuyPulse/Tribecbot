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

    static {
        instance = new IntakeImpl();
    }

    public static Intake getInstance() {
        return instance;
    }

    public enum PivotState {
        DEPLOY(Settings.Intake.PIVOT_DEPLOY_ANGLE),
        STOW(Settings.Intake.PIVOT_STOW_ANGLE);

        private final Rotation2d targetAngle;

        private PivotState(Rotation2d targetAngle) {
            this.targetAngle = targetAngle;
        }

        public Rotation2d getTargetAngle() {
            return targetAngle;
        }
    }

    public enum RollerState {
        INTAKE(1.0),
        OUTTAKE(-1.0),
        STOP(0.0);

        private final double targetDutyCycle;

        private RollerState(double targetDutyCycle) {
            this.targetDutyCycle = targetDutyCycle;
        }

        public double getTargetDutyCycle() {
            return targetDutyCycle;
        }
    }

    private PivotState pivotState;
    private RollerState rollerState;

    protected Intake() {
        this.pivotState = PivotState.STOW;
        this.rollerState = RollerState.STOP;
    }

    public PivotState getPivotState() {
        return pivotState;
    }

    public void setPivotState(PivotState state) {
        this.pivotState = state;
        setPivotVoltageOverride(Optional.empty());
    }

    public RollerState getRollerState() {
        return rollerState;
    }

    public void setRollerState(RollerState state) {
        this.rollerState = state;
    }

    public abstract boolean pivotAtTolerance();
    public abstract Rotation2d getPivotAngle();
    public abstract void setPivotVoltageOverride(Optional<Double> voltage);
    public abstract SysIdRoutine getPivotSysIdRoutine();

    @Override
    public void periodic() {
        SmartDashboard.putString("Intake/Pivot State", getPivotState().toString());
        SmartDashboard.putString("Intake/Roller State", getRollerState().toString());

        SmartDashboard.putNumber("Intake/Current Angle (deg)", getPivotAngle().getDegrees());
        SmartDashboard.putNumber("Intake/Target Angle (deg)", getPivotState().getTargetAngle().getDegrees());

        SmartDashboard.putNumber("Intake/Target Duty Cycle", getRollerState().getTargetDutyCycle());

        SmartDashboard.putBoolean("Intake/Pivot At Tolerance?", pivotAtTolerance());
    }
    
}
