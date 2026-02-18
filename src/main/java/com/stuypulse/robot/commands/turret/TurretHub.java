package com.stuypulse.robot.commands.turret;

import com.stuypulse.robot.subsystems.turret.Turret.TurretState;

public class TurretHub extends TurretSetState {
    public TurretHub() {
        super(TurretState.HUB);
    }
}