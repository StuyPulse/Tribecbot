package com.stuypulse.robot.commands.turret;

import com.stuypulse.robot.subsystems.turret.Turret.TurretState;

public class TurretZero extends TurretSetState {
    public TurretZero() {
        super(TurretState.ZERO);
    }
}