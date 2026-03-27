/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.SysId;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.Optional;

public class IntakeSim extends Intake {

    private static final double ARM_LENGTH_METERS = 0.4;
    private static final double ARM_MASS_KG = 2.0;
    private static final double MOI = SingleJointedArmSim.estimateMOI(ARM_LENGTH_METERS, ARM_MASS_KG);

    private final SingleJointedArmSim pivotSim;
    private final LinearSystemLoop<N2, N1, N2> pivotController;

    private final LinearSystemSim<N1, N1, N1> rollerLeaderSim;
    private final LinearSystemSim<N1, N1, N1> rollerFollowerSim;
    private final LinearSystemLoop<N1, N1, N1> rollerLeaderController;
    private final LinearSystemLoop<N1, N1, N1> rollerFollowerController;

    private Optional<Double> pivotVoltageOverride;

    public IntakeSim() {
        LinearSystem<N2, N1, N2> pivotSystem = LinearSystemId.createSingleJointedArmSystem(
            DCMotor.getKrakenX60(1),
            MOI,
            Settings.Intake.GEAR_RATIO
        );

        KalmanFilter<N2, N1, N2> kalmanFilter = new KalmanFilter<>(
            Nat.N2(),
            Nat.N2(),
            pivotSystem,
            VecBuilder.fill(3.0, 3.0),
            VecBuilder.fill(0.01, 0.01),
            Settings.DT
        );

        LinearQuadraticRegulator<N2, N1, N2> lqr = new LinearQuadraticRegulator<>(
            pivotSystem,
            VecBuilder.fill(0.00001, 100),
            VecBuilder.fill(12),
            Settings.DT
        );

        pivotController = new LinearSystemLoop<>(pivotSystem, lqr, kalmanFilter, 12.0, Settings.DT);

        pivotSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            Settings.Intake.GEAR_RATIO,
            MOI,
            ARM_LENGTH_METERS,
            Settings.Intake.PIVOT_MIN_ANGLE.getRadians(),
            Settings.Intake.PIVOT_MAX_ANGLE.getRadians(),
            true,
            Settings.Intake.PIVOT_MAX_ANGLE.getRadians() // start stowed
        );

        LinearSystem<N1, N1, N1> rollerSystem = LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60(1), 0.01, 1.0
        );

        rollerLeaderSim = new LinearSystemSim<>(rollerSystem);
        rollerFollowerSim = new LinearSystemSim<>(rollerSystem);

        LinearQuadraticRegulator<N1, N1, N1> rollerLQR = new LinearQuadraticRegulator<>(
            rollerSystem,
            VecBuilder.fill(8.0),
            VecBuilder.fill(12.0),
            Settings.DT
        );

        KalmanFilter<N1, N1, N1> rollerLeaderKalman = new KalmanFilter<>(
            Nat.N1(), Nat.N1(),
            rollerSystem,
            VecBuilder.fill(3.0),
            VecBuilder.fill(0.01),
            Settings.DT
        );

        KalmanFilter<N1, N1, N1> rollerFollowerKalman = new KalmanFilter<>(
            Nat.N1(), Nat.N1(),
            rollerSystem,
            VecBuilder.fill(3.0),
            VecBuilder.fill(0.01),
            Settings.DT
        );

        rollerLeaderController   = new LinearSystemLoop<>(rollerSystem, rollerLQR, rollerLeaderKalman,   12.0, Settings.DT);
        rollerFollowerController = new LinearSystemLoop<>(rollerSystem, rollerLQR, rollerFollowerKalman, 12.0, Settings.DT);

        pivotVoltageOverride = Optional.empty();
    }

    @Override
    public Rotation2d getPivotAngle() {
        return Rotation2d.fromRadians(pivotSim.getAngleRads());
    }

    @Override
    public boolean pivotAtTolerance() {
        double error = getPivotAngle().minus(getPivotState().getTargetAngle()).getRotations();
        return Math.abs(error) < Settings.Intake.PIVOT_ANGLE_TOLERANCE.getRotations();
    }

    @Override
    public boolean pivotStalling() {
        return false;
    }

    @Override
    public void seedPivotStowed() {
        pivotSim.setState(Settings.Intake.PIVOT_MAX_ANGLE.getRadians(), 0.0);
    }

    @Override
    public void seedPivotDeployed() {
        pivotSim.setState(Settings.Intake.PIVOT_MIN_ANGLE.getRadians(), 0.0);
    }

    @Override
    public void periodic() {
        super.periodic();

        PivotState pivotState = getPivotState();

        double targetRadPerSec = getRollerState().getTargetDutyCycle() * 2.0 * Math.PI / 60.0 * 6000.0;

        rollerLeaderController.setNextR(VecBuilder.fill(targetRadPerSec));
        rollerLeaderController.correct(VecBuilder.fill(rollerLeaderSim.getOutput(0)));
        rollerLeaderController.predict(Settings.DT);

        rollerFollowerController.setNextR(VecBuilder.fill(targetRadPerSec));
        rollerFollowerController.correct(VecBuilder.fill(rollerFollowerSim.getOutput(0)));
        rollerFollowerController.predict(Settings.DT);

        if (EnabledSubsystems.INTAKE.get()) {
            if (pivotVoltageOverride.isPresent()) {
                pivotSim.setInputVoltage(pivotVoltageOverride.get());
            } else {
                pivotController.setNextR(VecBuilder.fill(pivotState.getTargetAngle().getRadians(), 0.0));
                pivotController.correct(VecBuilder.fill(pivotSim.getAngleRads(), pivotSim.getVelocityRadPerSec()));
                pivotController.predict(Settings.DT);
                pivotSim.setInputVoltage(pivotController.getU(0));
            }

            if (pivotState == PivotState.DEPLOY && getPivotAngle().getDegrees() <= Settings.Intake.THRESHOLD_TO_START_ROLLERS.getDegrees()) {
                rollerLeaderSim.setInput(rollerLeaderController.getU(0));
                rollerFollowerSim.setInput(rollerFollowerController.getU(0));
            } else {
                rollerLeaderSim.setInput(0.0);
                rollerFollowerSim.setInput(0.0);
            }
        } else {
            pivotSim.setInputVoltage(0.0);
            rollerLeaderSim.setInput(0.0);
            rollerFollowerSim.setInput(0.0);
        }

        pivotSim.update(Settings.DT);
        rollerLeaderSim.update(Settings.DT);
        rollerFollowerSim.update(Settings.DT);

        if (Settings.DEBUG_MODE.get()) {
            SmartDashboard.putNumber("Intake/Sim Pivot Angle (deg)", getPivotAngle().getDegrees());
            SmartDashboard.putNumber("Intake/Sim Pivot Velocity (deg per s)", Units.radiansToDegrees(pivotSim.getVelocityRadPerSec()));
            SmartDashboard.putNumber("Intake/Sim Roller Leader Velocity (RPM)", rollerLeaderSim.getOutput(0) * 60.0 / (2.0 * Math.PI));
            SmartDashboard.putNumber("Intake/Sim Roller Follower Velocity (RPM)", rollerFollowerSim.getOutput(0) * 60.0 / (2.0 * Math.PI));
        }
    }

    @Override
    public void setPivotVoltageOverride(Optional<Double> voltage) {
        this.pivotVoltageOverride = voltage;
    }

    @Override
    public SysIdRoutine getPivotSysIdRoutine() {
        return SysId.getRoutine(
            2,
            6,
            "Intake Pivot",
            voltage -> setPivotVoltageOverride(Optional.of(voltage)),
            () -> getPivotAngle().getRotations(),
            () -> pivotSim.getVelocityRadPerSec(),
            () -> pivotVoltageOverride.orElse(0.0),
            getInstance()
        );
    }

    @Override
    public double getCurrentDraw() {
        return 0;
    }
}