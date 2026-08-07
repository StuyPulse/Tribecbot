package com.stuypulse.robot.commands.auton.bline;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.handoff.HandoffRun;
import com.stuypulse.robot.commands.handoff.HandoffStop;
import com.stuypulse.robot.commands.intake.IntakeAutoDigest;
import com.stuypulse.robot.commands.intake.IntakeDeploy;
import com.stuypulse.robot.commands.spindexer.SpindexerRun;
import com.stuypulse.robot.commands.spindexer.SpindexerStop;
import com.stuypulse.robot.commands.superstructure.SuperstructureAutoInterpolation;
import com.stuypulse.robot.commands.superstructure.SuperstructureSOTM;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.BLinePathUtil;

import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.util.Set;
import java.util.function.Consumer;

public class LeftTwoCornerBLine extends SequentialCommandGroup {

    private static final double HANDOFF_THRESHOLD_METERS = 0.15; // tune per-path

    public LeftTwoCornerBLine(String... pathNames) {

        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

        addCommands(

            Commands.defer(() -> new WaitCommand(RobotContainer.getWaitTimeOne()), Set.of()),

            // NZ Trip 1 — first path, pose reset happens here
            followUntil(swerve, pathNames[0], HANDOFF_THRESHOLD_METERS, swerve::resetPose).alongWith(
                    new WaitCommand(0.2).andThen(new IntakeDeploy())
                ),

            // Trip 1 To Score
            followUntil(swerve, pathNames[1], HANDOFF_THRESHOLD_METERS, pose -> {}).alongWith(
                    new SuperstructureAutoInterpolation()
                ),
            new SuperstructureSOTM(),
            new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
            new ParallelCommandGroup(
                new HandoffRun(),
                new SpindexerRun(),
                new WaitCommand(0.5)
                    .andThen(new IntakeAutoDigest().until(() -> Superstructure.getInstance().isHopperEmpty()).withTimeout(15.0)),
                new WaitCommand(1.0).andThen(
                    new WaitUntilCommand(() -> Superstructure.getInstance().isHopperEmpty()).withTimeout(4.0))
            ),
            new SuperstructureAutoInterpolation().alongWith(new IntakeDeploy()),

            // NZ Trip 2
            new ParallelCommandGroup(
                followUntil(swerve, pathNames[2], HANDOFF_THRESHOLD_METERS, pose -> {}),
                new HandoffStop(),
                new SpindexerStop()
            ),

            new SuperstructureSOTM(),
            new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
            new ParallelCommandGroup(
                followUntil(swerve, pathNames[3], HANDOFF_THRESHOLD_METERS, pose -> {}),
                new HandoffRun(),
                new SpindexerRun(),
                new WaitCommand(0.5)
                    .andThen(new IntakeAutoDigest().until(() -> Superstructure.getInstance().isHopperEmpty()).withTimeout(15.0)),
                new WaitUntilCommand(() -> Superstructure.getInstance().isHopperEmpty()).withTimeout(15.0)
            ),

            swerve.getPathBuilder()
                .withPoseReset(pose -> {})
                .build(new Path(BLinePathUtil.PATHS_DIR, pathNames[4])).alongWith(new IntakeDeploy())

        );

    }

    private static Command followUntil(CommandSwerveDrivetrain swerve, String pathName, double thresholdMeters, Consumer<Pose2d> poseReset) {
        FollowPath path = (FollowPath) swerve.getPathBuilder()
            .withPoseReset(poseReset)
            .build(new Path(BLinePathUtil.PATHS_DIR, pathName));

        return path.raceWith(
            new WaitUntilCommand(() -> path.getRemainingPathDistanceMeters() < thresholdMeters)
        );
    }

}