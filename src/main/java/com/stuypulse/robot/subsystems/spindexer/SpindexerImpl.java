package com.stuypulse.robot.subsystems.spindexer;

import java.util.Optional;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.SysId;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SpindexerImpl extends Spindexer {
    private final TalonFX leadMotor;
    private final TalonFX followerMotor;

    private final VelocityVoltage controller;
    private final Follower follower;

    private Optional<Double> voltageOverride;

    public SpindexerImpl() {
        leadMotor = new TalonFX(Ports.Spindexer.SPINDEXER_LEAD_MOTOR);
        followerMotor = new TalonFX(Ports.Spindexer.SPINDEXER_FOLLOW_MOTOR);

        Motors.Spindexer.MOTOR_CONFIG.configure(leadMotor);
        Motors.Spindexer.MOTOR_CONFIG.configure(followerMotor);

        follower = new Follower(Ports.Spindexer.SPINDEXER_LEAD_MOTOR, MotorAlignmentValue.Opposed);

        controller = new VelocityVoltage(getTargetRPM());

        voltageOverride = Optional.empty();
    }

    public double getCurrentLeadMotorRPM() {
        return leadMotor.getVelocity().getValueAsDouble() * Settings.Spindexer.SECONDS_IN_A_MINUTE;
    }

    public double getCurrentFollowerMotorRPM() {
        return followerMotor.getVelocity().getValueAsDouble() * Settings.Spindexer.SECONDS_IN_A_MINUTE;
    }

    public boolean atTolerance() {
        return Math.abs(getCurrentLeadMotorRPM() - getTargetRPM()) <= Settings.Spindexer.SPINDEXER_TOLERANCE;
    }

    @Override
    public void periodic() {
        super.periodic();
        if (Settings.EnabledSubsystems.SPINDEXER.get()) {
            if (voltageOverride.isPresent()){
                leadMotor.setVoltage(voltageOverride.get());
                followerMotor.setControl(follower);
            } else {
                if (getState() == SpindexerState.STOP){
                    leadMotor.stopMotor();
                    followerMotor.stopMotor();
                } else {
                    leadMotor.setControl(controller.withVelocity(getTargetRPM() / Settings.Spindexer.SECONDS_IN_A_MINUTE));
                    followerMotor.setControl(follower);
                }
            }
        }

        if (Settings.DEBUG_MODE) {
            SmartDashboard.putNumber("Spindexer/Lead Motor RPM", getCurrentLeadMotorRPM());
            SmartDashboard.putNumber("Spindexer/Follower Motor RPM", getCurrentFollowerMotorRPM());

            SmartDashboard.putBoolean("Spindexer/At Tolerance", atTolerance());

            SmartDashboard.putNumber("Spindexer/Lead Current (amps)", leadMotor.getStatorCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Spindexer/Lead Voltage", leadMotor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Spindexer/Lead Supply Current", leadMotor.getSupplyCurrent().getValueAsDouble());

            SmartDashboard.putNumber("Spindexer/Follower Current (amps)", followerMotor.getStatorCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Spindexer/Follower Voltage", followerMotor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Spindexer/Follower Supply Current", followerMotor.getSupplyCurrent().getValueAsDouble());
        }
    }

    @Override
    public void setVoltageOverride(Optional<Double> voltage) {
        this.voltageOverride = voltage;
    }

    @Override
    public SysIdRoutine getSysIdRoutine() {
        return SysId.getRoutine(
            1,
            2,
            "Spindexer",
            voltage -> setVoltageOverride(Optional.of(voltage)),
            () -> leadMotor.getPosition().getValueAsDouble(),
            () -> leadMotor.getVelocity().getValueAsDouble(),
            () -> leadMotor.getMotorVoltage().getValueAsDouble(),
            getInstance()
        );
    }
}