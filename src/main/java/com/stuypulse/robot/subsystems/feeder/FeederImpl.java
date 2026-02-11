package com.stuypulse.robot.subsystems.feeder;

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

public class FeederImpl extends Feeder {
    private final TalonFX leadMotor;
    private final TalonFX followerMotor;

    private final VelocityVoltage leadMotorController;

    private Optional<Double> voltageOverride;

    public FeederImpl() {
        leadMotor = new TalonFX(Ports.Feeder.FEEDER_LEADER);
        Motors.Feeder.motorConfig.configure(leadMotor);

        followerMotor = new TalonFX(Ports.Feeder.FEEDER_FOLLOWER);
        Motors.Feeder.motorConfig.configure(followerMotor);

        leadMotorController = new VelocityVoltage(0.0);
        followerMotor.setControl(new Follower(Ports.Feeder.FEEDER_LEADER, MotorAlignmentValue.Aligned));

        voltageOverride = Optional.empty();
    }

    @Override
    public SysIdRoutine getSysIdRoutine() {
        return SysId.getRoutine(
                1, // ramp rate
                2, // step voltage
                "Feeder",
                voltage -> setVoltageOverride(Optional.of(voltage)),
                () -> leadMotor.getPosition().getValueAsDouble(), // position supplier?
                () -> leadMotor.getVelocity().getValueAsDouble(),
                () -> leadMotor.getMotorVoltage().getValueAsDouble(),
                getInstance());
    }

    /**
     * @return lead motor RPM
     */
    public double getCurrentLeadMotorRPM() {
        return leadMotor.getVelocity().getValueAsDouble() * Settings.Feeder.SECONDS_IN_A_MINUTE;
    }

    /**
     * @return follower motor RPM
     */
    public double getCurrentFollowerMotorRPM() {
        return followerMotor.getVelocity().getValueAsDouble() * Settings.Feeder.SECONDS_IN_A_MINUTE;
    }

    /**
     * @return the voltage override if it is present; otherwise, return 0
     */
    @Override
    public double getVoltageOverride() {
        if (voltageOverride.isPresent())
            return voltageOverride.get();
        else
            return 0;
    }

    /**
     * @param voltage as an Optional<Double> to handle the case of null values;
     *                in the case that voltage is not null, voltageOverride will be
     *                set to thhe passed in voltage
     */
    @Override
    public void setVoltageOverride(Optional<Double> voltage) {
        this.voltageOverride = voltage;
    }

    @Override
    public void periodic() {
        super.periodic();

        if (!Settings.EnabledSubsystems.FEEDER.get()) {
            if (voltageOverride.isPresent()) {
                leadMotor.setVoltage(voltageOverride.get());
            } else {
                if (getFeederState() == FeederState.STOP) {
                    leadMotor.stopMotor();
                } else {
                    leadMotor.setControl(
                            leadMotorController.withVelocity(getTargetRPM() / Settings.Feeder.SECONDS_IN_A_MINUTE));
                }
            }

        } else {
            leadMotor
                .setControl(leadMotorController.withVelocity(getTargetRPM() / Settings.Feeder.SECONDS_IN_A_MINUTE));
        }

        if (Settings.DEBUG_MODE) {
            SmartDashboard.putNumber("Feeder/Lead Motor Speed", getCurrentLeadMotorRPM());
            SmartDashboard.putNumber("Feeder/Follower Motor RPM", getCurrentFollowerMotorRPM());
        }
    }
}