package com.stuypulse.robot.commands.auton.bline;

import java.util.Set;
import java.util.function.Consumer;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.BLinePathUtil;

import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class TwoMeterBLine extends SequentialCommandGroup {
    private static final double HANDOFF_THRESHOLD_METERS = 0.15; // tune per-path

    public TwoMeterBLine(String... pathNames) {

        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

        addCommands(

            Commands.defer(() -> new WaitCommand(RobotContainer.getWaitTimeOne()), Set.of()),

            // NZ Trip 1 — first path, pose reset happens here, raced against threshold
            followUntil(swerve, pathNames[0], HANDOFF_THRESHOLD_METERS, swerve::resetPose)
        );

    }

    private static Command followUntil(CommandSwerveDrivetrain swerve, String pathName, double thresholdMeters, Consumer<Pose2d> poseReset) {
        FollowPath path = (FollowPath) swerve.getPathBuilder()
            .withPoseReset(poseReset)
            .build(new Path(pathName));

        return path.raceWith(
            new WaitUntilCommand(() -> path.getRemainingPathDistanceMeters() < thresholdMeters)
        );
    }
}
