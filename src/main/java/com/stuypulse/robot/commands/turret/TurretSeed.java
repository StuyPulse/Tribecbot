package com.stuypulse.robot.commands.turret;

import com.stuypulse.robot.subsystems.turret.Turret;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class TurretSeed extends InstantCommand {
    private Turret turret;

    public TurretSeed() {
        turret = Turret.getInstance();

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.seedTurret();
    }
}