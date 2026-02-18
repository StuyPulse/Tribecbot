package com.stuypulse.robot.commands.turret;

import com.stuypulse.robot.subsystems.turret.Turret.TurretState;

public class TurretFerry extends TurretSetState {
    public TurretFerry() {
        super(TurretState.FERRYING);
    }
}