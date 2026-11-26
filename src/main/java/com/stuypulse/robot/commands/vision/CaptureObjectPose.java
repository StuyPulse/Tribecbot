package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.vision.LimelightVision;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class CaptureObjectPose extends InstantCommand {
    private LimelightVision vision;
    private CommandSwerveDrivetrain swerve;

    public CaptureObjectPose() {
        vision = LimelightVision.getInstance();
        swerve = CommandSwerveDrivetrain.getInstance();
    }

    @Override
    public void initialize() {
        vision.setPoseInUse(swerve.getPose().getRotation());
    }
}