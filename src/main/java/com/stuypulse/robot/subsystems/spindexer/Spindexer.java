package com.stuypulse.robot.subsystems.spindexer;

import java.util.Optional;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.SpindexerInterpolation;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public abstract class Spindexer extends SubsystemBase {
    private static final Spindexer instance;
    private SpindexerState spindexerState;

    static {
        instance = new SpindexerImpl();
    }

    public static Spindexer getInstance() {
        return instance;
    }

    public enum SpindexerState {
        STOP,
        DYNAMIC, // In case we need for future: interpolates RPM based on distance to hub
        FORWARD,
        REVERSE;
    }

    public double getTargetRPM() {
        return switch (getState()) {
            case STOP -> 0;
            case DYNAMIC -> getRPMBasedOnDistance(); 
            case FORWARD -> Settings.Spindexer.FORWARD_RPM;
            case REVERSE -> Settings.Spindexer.REVERSE_RPM;
        };
    }

    public Spindexer() {
        spindexerState = SpindexerState.STOP;
    }

    public SpindexerState getState() {
        return spindexerState;
    }

    public void setState(SpindexerState state) {
        this.spindexerState = state;
    }

    /**
     * @return target RPM interpolated based on distance
     */
    public double getRPMBasedOnDistance() {
        Translation2d hubPos = Field.getHubPose().getTranslation();
        Translation2d robotPos = CommandSwerveDrivetrain.getInstance().getPose().getTranslation();
        double distance = hubPos.getDistance(robotPos);
        return SpindexerInterpolation.getRPM(distance);
    }

    public abstract SysIdRoutine getSysIdRoutine();
    public abstract void setVoltageOverride(Optional<Double> voltage);

    @Override
    public void periodic() {
        if (Settings.DEBUG_MODE) {
            SmartDashboard.putString("Spindexer/State", getState().name());
            SmartDashboard.putString("States/Spindexer", getState().name());

            SmartDashboard.putNumber("Spindexer/Target RPM", getTargetRPM());
            SmartDashboard.putNumber("Spindexer/Interpolated RPM Based on Distance", getRPMBasedOnDistance());
        }
    }
}