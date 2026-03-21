package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SwerveTeleopInit extends InstantCommand {
    private final CommandSwerveDrivetrain swerve;

    public SwerveTeleopInit() {
        swerve = CommandSwerveDrivetrain.getInstance();        
    }

    @Override
    public void initialize() {
        swerve.teleopInit();
    }
}
