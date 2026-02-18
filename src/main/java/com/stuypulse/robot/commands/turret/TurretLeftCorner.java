package com.stuypulse.robot.commands.turret;

import com.stuypulse.robot.subsystems.turret.Turret.TurretState;

public class TurretLeftCorner extends TurretSetState {
    public TurretLeftCorner() {
        super(TurretState.LEFT_CORNER);
    }
}