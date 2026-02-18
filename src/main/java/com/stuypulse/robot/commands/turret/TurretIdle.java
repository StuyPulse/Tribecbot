package com.stuypulse.robot.commands.turret;

import com.stuypulse.robot.subsystems.turret.Turret.TurretState;

public class TurretIdle extends TurretSetState {
    public TurretIdle() {
        super(TurretState.IDLE);
    }
}