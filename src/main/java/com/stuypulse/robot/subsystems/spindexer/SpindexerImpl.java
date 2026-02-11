package com.stuypulse.robot.subsystems.spindexer;

import java.util.Optional;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.SpindexerInterpolation;
import com.stuypulse.robot.util.SysId;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SpindexerImpl extends Spindexer {
    private final TalonFX leadMotor;
    private final TalonFX followerMotor;

    private final VelocityVoltage leadMotorController;

    private Optional<Double> voltageOverride;

    public SpindexerImpl() {
        leadMotor = new TalonFX(Ports.Spindexer.SPINDEXER_LEAD_MOTOR);
        followerMotor = new TalonFX(Ports.Spindexer.SPINDEXER_FOLLOW_MOTOR);

        Motors.Spindexer.MOTOR_CONFIG.configure(leadMotor);
        Motors.Spindexer.MOTOR_CONFIG.configure(followerMotor);

        followerMotor.setControl(new Follower(Ports.Spindexer.SPINDEXER_LEAD_MOTOR, MotorAlignmentValue.Opposed));

        leadMotorController = new VelocityVoltage(0.0);

        voltageOverride = Optional.empty();
    }

    @Override
    public SysIdRoutine getSysIdRoutine() {
        return SysId.getRoutine(
            1, // ramp rate
            2, // step voltage
            "Spindexer",
            voltage -> setVoltageOverride(Optional.of(voltage)),
            () -> leadMotor.getPosition().getValueAsDouble(), // position supplier?
            () -> leadMotor.getVelocity().getValueAsDouble(),
            () -> leadMotor.getMotorVoltage().getValueAsDouble(),
            getInstance()
        );
    }

    /**
     * @return RPM of the lead motor for the spindexer
     */
    public double getCurrentLeadMotorRPM() {
        return leadMotor.getVelocity().getValueAsDouble() * Settings.Spindexer.SECONDS_IN_A_MINUTE;
    }

    /**
     * @return RPM of the follower motor for the spindexer
     */
    public double getCurrentFollowerMotorRPM() {
        return followerMotor.getVelocity().getValueAsDouble() * Settings.Spindexer.SECONDS_IN_A_MINUTE;
    }

    /**
     * @return RPM based on distance; interpolated from data points
     */
    @Override
    public double getRPMBasedOnDistance() {
        Translation2d hubPos = Field.getHubCenterPose().getTranslation();
        Translation2d robotPos = CommandSwerveDrivetrain.getInstance().getPose().getTranslation();
        double distance = hubPos.getDistance(robotPos);
        return SpindexerInterpolation.getRPM(distance);
    }

    /**
     * @return a boolean if lead motor is within tolerance range
     */
    public boolean atTargetRPM() {
        return Math.abs(getCurrentLeadMotorRPM() - getTargetRPM()) <= Settings.Spindexer.SPINDEXER_TOLERANCE;
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
        if (Settings.EnabledSubsystems.SPINDEXER.get()) {
            if (voltageOverride.isPresent()){
                leadMotor.setVoltage(voltageOverride.get());
            }
            else {
                if (getSpindexerState() == SpindexerState.STOP){
                    leadMotor.stopMotor(); 
                } else {
                    leadMotor.setControl(
                        leadMotorController.withVelocity(getTargetRPM() / Settings.Spindexer.SECONDS_IN_A_MINUTE));
                }
            }
        }

        if (Settings.DEBUG_MODE) {
            SmartDashboard.putNumber("Spindexer/Lead Motor Speed", getCurrentLeadMotorRPM());
            SmartDashboard.putNumber("Spindexer/Follower Motor RPM", getCurrentFollowerMotorRPM());
            SmartDashboard.putNumber("Spindexer/Est. RPM Based on Distance", getRPMBasedOnDistance());
            SmartDashboard.putBoolean("Spindexer/At Target RPM", atTargetRPM());
        }
    }
}