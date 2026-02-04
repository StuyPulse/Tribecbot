package com.stuypulse.robot.subsystems.hoodedshooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodedShooter extends SubsystemBase {
    private static final HoodedShooter instance;
    private HoodedShooterState state;

    static {
        instance = new HoodedShooterImpl();
    }

    public static HoodedShooter getInstance() {
        return instance;
    }

    public enum HoodedShooterState {
        STOW();

        private Rotation2d targetAngle;
        private double targetRPM;
        
        private HoodedShooterState() {

        }
    }

    @Override
    public void periodic() {

    } 
}
