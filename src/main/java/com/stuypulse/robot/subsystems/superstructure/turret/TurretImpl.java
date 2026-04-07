/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.superstructure.turret;

import java.util.Optional;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.RobotMode;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.DriverConstants;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.MasterLogger;
import com.stuypulse.robot.util.MotorLogger;
import com.stuypulse.robot.util.MotorLogger.SubsystemName;
import com.stuypulse.robot.util.MotorLogger.ValueKey;
import com.stuypulse.robot.util.PhoenixUtil.PublishingDestination;
import com.stuypulse.robot.util.SysId;
import com.stuypulse.robot.util.superstructure.TurretAngleCalculator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class TurretImpl extends Turret {
    private final Motors.TalonFXConfig turretConfig;
    private final Motors.CANCoderConfig encoder17tConfig;
    private final Motors.CANCoderConfig encoder18tConfig;

    private final TalonFX turretMotor;
    private final CANcoder encoder17t;
    private final CANcoder encoder18t;

    private boolean hasUsedAbsoluteEncoder;
    private boolean hasInitializedFilter;

    private Optional<Double> voltageOverride;
    private final PositionVoltage controller;

    private double prevActualTargetAngle;
    private boolean isWrapping;
    
    private final MotorLogger turretLogger;
    private final MasterLogger turretMaster;

    public TurretImpl() {
        turretConfig = new Motors.TalonFXConfig()
                .withInvertedValue(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)

                .withSupplyCurrentLimitAmps(80)
                .withStatorCurrentLimitEnabled(false)
                .withRampRate(0.25)

                .withPIDConstants(Gains.Superstructure.Turret.slot0.kP, Gains.Superstructure.Turret.slot0.kI,
                        Gains.Superstructure.Turret.slot0.kD, 0)
                .withFFConstants(Gains.Superstructure.Turret.slot0.kS, Gains.Superstructure.Turret.slot0.kV,
                        Gains.Superstructure.Turret.slot0.kA, 0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign, 0)

                .withPIDConstants(0, 0, 10.0, 2)
                .withFFConstants(Gains.Superstructure.Turret.slot0.kS, Gains.Superstructure.Turret.slot0.kV,
                        Gains.Superstructure.Turret.slot0.kA, 2)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign, 2)

                .withPIDConstants(Gains.Superstructure.Turret.slot1.kP.get(),
                        Gains.Superstructure.Turret.slot1.kI.get(), Gains.Superstructure.Turret.slot1.kD.get(), 1)
                .withFFConstants(Gains.Superstructure.Turret.slot1.kS.get(), Gains.Superstructure.Turret.slot1.kV.get(),
                        Gains.Superstructure.Turret.slot1.kA.get(), 1)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign, 1)

                .withSensorToMechanismRatio(Settings.Superstructure.Turret.GEAR_RATIO_MOTOR_TO_MECH)

                .withSoftLimits(
                        false, false,
                        Settings.Superstructure.Turret.SoftwareLimit.FORWARD_MAX_ROTATIONS,
                        Settings.Superstructure.Turret.SoftwareLimit.BACKWARDS_MAX_ROTATIONS);

        encoder17tConfig = new Motors.CANCoderConfig()
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                .withMagnetOffset(Settings.Superstructure.Turret.Encoder17t.OFFSET.getRotations())
                .withAbsoluteSensorDiscontinuityPoint(1.0);

        encoder18tConfig = new Motors.CANCoderConfig()
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                .withMagnetOffset(Settings.Superstructure.Turret.Encoder18t.OFFSET.getRotations())
                .withAbsoluteSensorDiscontinuityPoint(1.0);

        turretMotor = new TalonFX(Ports.Superstructure.Turret.MOTOR, Ports.RIO);
        encoder17t = new CANcoder(Ports.Superstructure.Turret.ENCODER17T, Ports.RIO);
        encoder18t = new CANcoder(Ports.Superstructure.Turret.ENCODER18T, Ports.RIO);

        turretConfig.configure(turretMotor);
        encoder17tConfig.configure(encoder17t);
        encoder18tConfig.configure(encoder18t);

        hasUsedAbsoluteEncoder = false;
        hasInitializedFilter = false;
        voltageOverride = Optional.empty();
        prevActualTargetAngle = getTargetAngle().getDegrees();

        controller = new PositionVoltage(getTargetAngle().getRotations()).withEnableFOC(true);

        turretMotor.getClosedLoopError().setUpdateFrequency(1000.0);

        turretLogger = new MotorLogger(SubsystemName.Turret, "Motor", turretMotor, Ports.Superstructure.Turret.MOTOR, PublishingDestination.RIO);

        turretMaster = new MasterLogger(turretLogger);
        turretMaster.addEncoder(encoder17t, "Encoder 17t");
        turretMaster.addEncoder(encoder18t, "Encoder 18t");
    }

    private Rotation2d getEncoderPos17t() {
        return Rotation2d.fromRotations(turretMaster.encoderValues.get("Encoder 17t").getValueAsDouble());
    }

    private Rotation2d getEncoderPos18t() {
        return Rotation2d.fromRotations(turretMaster.encoderValues.get("Encoder 18t").getValueAsDouble());
    }

    public Rotation2d getVectorSpaceAngle() {
        return TurretAngleCalculator.getAbsoluteAngle(getEncoderPos17t().getDegrees(), getEncoderPos18t().getDegrees());
    }

    public void zeroEncoders() {
        double encoderPos17T = turretMaster.encoderValues.get("Encoder 17t").getValueAsDouble();
        double encoderPos18T = turretMaster.encoderValues.get("Encoder 18t").getValueAsDouble();

        encoder17t.getConfigurator().refresh(encoder17tConfig.getConfiguration().MagnetSensor);
        encoder18t.getConfigurator().refresh(encoder18tConfig.getConfiguration().MagnetSensor);

        double currentOffset17T = encoder17tConfig.getConfiguration().MagnetSensor.MagnetOffset;
        double currentOffset18T = encoder18tConfig.getConfiguration().MagnetSensor.MagnetOffset;

        double newOffset17T = currentOffset17T - encoderPos17T;
        double newOffset18T = currentOffset18T - encoderPos18T;

        encoder17tConfig.withMagnetOffset(newOffset17T);
        encoder18tConfig.withMagnetOffset(newOffset18T);

        encoder17tConfig.configure(encoder17t);
        encoder18tConfig.configure(encoder18t);
    }

    public void seedTurret() {
        turretMotor.setPosition(getVectorSpaceAngle().getRotations());
    }

    public boolean isWrapping() {
        return isWrapping;
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(turretMaster.getMotorSignalMap(turretLogger).get(ValueKey.Position.toString()).getValueAsDouble());
    }

    private double getDelta(double target, double current) {
        double delta = (target - current) % 360;

        if (delta > 180.0) {
            delta -= 360;
        } else if (delta < -180) {
            delta += 360;
        }

        if (current + delta > Settings.Superstructure.Turret.RANGE_CW) {
            return delta - 360;
        }
        if (current + delta < Settings.Superstructure.Turret.RANGE_CCW) {
            return delta + 360;
        }

        return delta;
    }

    public double getWrappedTargetAngle() {
        double currentAngle = getAngle().getDegrees();
        return currentAngle + getDelta(getTargetAngle().getDegrees(), currentAngle);
    }


    @Override
    public void periodicAfterScheduler() {
        turretMaster.logEverything();

        super.periodicAfterScheduler();
        
        turretConfig.updateGainsConfig(
                turretMotor, 1,
                Gains.Superstructure.Turret.slot1.kP,
                Gains.Superstructure.Turret.slot1.kI,
                Gains.Superstructure.Turret.slot1.kD,
                Gains.Superstructure.Turret.slot1.kS,
                Gains.Superstructure.Turret.slot1.kV,
                Gains.Superstructure.Turret.slot1.kA);

        if (!hasUsedAbsoluteEncoder) {
            seedTurret();
            hasUsedAbsoluteEncoder = true;
        }

        double currentAngle = getAngle().getDegrees();
        double actualTargetAngle = currentAngle + getDelta(getTargetAngle().getDegrees(), currentAngle);

        if (!hasInitializedFilter) {
            prevActualTargetAngle = actualTargetAngle;
            hasInitializedFilter = true;
        }

        double delta = actualTargetAngle - prevActualTargetAngle;

        boolean deltaIsSignificant = Math.abs(delta)  >= Settings.Superstructure.Turret.SETPOINT_FILTER_THRESHOLD_DEG;

        boolean driverIsMoving = Math.abs(RobotContainer.driver.getLeftX()) > DriverConstants.Driver.Drive.DEADBAND ||
                Math.abs(RobotContainer.driver.getLeftY()) > DriverConstants.Driver.Drive.DEADBAND ||
                Math.abs(RobotContainer.driver.getRightX()) > DriverConstants.Driver.Drive.DEADBAND;

        if (deltaIsSignificant || driverIsMoving) {
            prevActualTargetAngle = actualTargetAngle;
        }

        isWrapping = Math.abs(getWrappedTargetAngle()
                - currentAngle) > Settings.Superstructure.Turret.GAIN_SWITCHING_THRESHOLD.getDegrees();
        int slot = 0;

        if (isWrapping) {
            slot = 1;
        }
        // else if (!deltaIsSignificant) {
        //     slot = 2;
        // }

        if (EnabledSubsystems.TURRET.get()) {
            if (voltageOverride.isPresent()) {
                turretMotor.setVoltage(voltageOverride.get());
            } 
            else {
                double omega = CommandSwerveDrivetrain.getInstance().getChassisSpeeds().omegaRadiansPerSecond;
                double omegaFF = Gains.Superstructure.Turret.kOmega.get() * omega;
                // double setpointVelocityRPS = delta / (360 * Settings.DT);
                // double translationFF = Gains.Superstructure.Turret.slot0.kV * setpointVelocityRPS;

                turretMotor.setControl(controller
                    .withPosition(prevActualTargetAngle / 360.0)
                    .withSlot(slot)
                    .withFeedForward(omegaFF /* + translationFF */)
                );
            }
        } else {
            turretMotor.stopMotor();
        }
        
        SmartDashboard.putNumber("Superstructure/Turret/Wrapped Target Angle (deg)", prevActualTargetAngle);

            if (Robot.getMode() == RobotMode.DISABLED && !DriverStation.isFMSAttached()) {
                SmartDashboard.putBoolean("Robot/CAN/Main/Turret 17t Encoder Connected? (ID "
                        + String.valueOf(Ports.Superstructure.Turret.ENCODER17T) + ")", encoder17t.isConnected());
                SmartDashboard.putBoolean("Robot/CAN/Main/Turret 18t Encoder Connected? (ID "
                        + String.valueOf(Ports.Superstructure.Turret.ENCODER18T) + ")", encoder18t.isConnected());
            }
        
        Robot.getEnergyUtil().logEnergyUsage(getName(), getCurrentDraw());
    }

    private void setVoltageOverride(Optional<Double> volts) {
        this.voltageOverride = volts;
    }

    @Override
    public SysIdRoutine getSysIdRoutine() {
        return SysId.getRoutine(
                2,
                6,
                "Turret",
                voltage -> setVoltageOverride(Optional.of(voltage)),
                () -> this.turretMotor.getPosition().getValueAsDouble(),
                () -> this.turretMotor.getVelocity().getValueAsDouble(),
                () -> this.turretMotor.getMotorVoltage().getValueAsDouble(),
                getInstance());
    }

    @Override
    public double getCurrentDraw() {
        return Double.max(0, turretMaster.getMotorSignalMap(turretLogger).get(ValueKey.Supply_Current.toString()).getValueAsDouble());
    }
}