package com.stuypulse.robot.commands.turret;

import com.stuypulse.robot.subsystems.turret.Turret.TurretState;

public class TurretRightCorner extends TurretSetState {
    public TurretRightCorner() {
        super(TurretState.RIGHT_CORNER);
    }
}