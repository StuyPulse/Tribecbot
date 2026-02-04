package com.stuypulse.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    private static final Turret instance;
    private TurretState state;

    static {
        instance = new TurretImpl();
    }

    public static Turret getInstance() {
        return instance;
    }

    public enum TurretState {
        STOP();
        
        private TurretState() {

        }
    }

    @Override
    public void periodic() {

    }
}
