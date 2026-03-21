package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SwerveAutonInit extends InstantCommand {
    private final CommandSwerveDrivetrain swerve;

    public SwerveAutonInit() {
        swerve = CommandSwerveDrivetrain.getInstance();        
    }

    @Override
    public void initialize() {
        swerve.autonInit();
    }
}
