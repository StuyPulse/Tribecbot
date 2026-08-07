package com.stuypulse.robot.commands.auton.bline;

import java.util.Set;
import java.util.function.Consumer;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.handoff.HandoffRun;
import com.stuypulse.robot.commands.handoff.HandoffStop;
import com.stuypulse.robot.commands.intake.IntakeAutoDigest;
import com.stuypulse.robot.commands.intake.IntakeDeploy;
import com.stuypulse.robot.commands.spindexer.SpindexerRun;
import com.stuypulse.robot.commands.spindexer.SpindexerStop;
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

public class LeftFollowBLine extends SequentialCommandGroup {

    private static final double HANDOFF_THRESHOLD_METERS = 0.15; // tune per-path

    public LeftFollowBLine(String... pathNames) {

        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

        addCommands(

            // Preloads
            new SuperstructureSOTM(),
            new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
            new ParallelCommandGroup(
                new HandoffRun(),
                new SpindexerRun(),
                Commands.defer(() -> new WaitCommand(RobotContainer.getWaitTimeOne() + 1.0), Set.of()),
                new WaitCommand(1.0).andThen(new IntakeDeploy())
            ),

            // To NZ — first actual path driven, so pose reset happens here
            new ParallelCommandGroup(
                new HandoffStop(),
                new SpindexerStop(),
                followUntil(swerve, pathNames[0], HANDOFF_THRESHOLD_METERS, swerve::resetPose)
            ),

            Commands.defer(() -> new WaitCommand(RobotContainer.getWaitTimeTwo()), Set.of()),

            // Back
            new ParallelCommandGroup(
                followUntil(swerve, pathNames[1], HANDOFF_THRESHOLD_METERS, pose -> {}),
                new WaitCommand(3.0).andThen(
                    new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance())
                        .andThen(
                            new ParallelCommandGroup(
                                new HandoffRun(),
                                new SpindexerRun()))
                )
            ),

            new HandoffStop().alongWith(new SpindexerStop()),

            swerve.getPathBuilder()
                .withPoseReset(pose -> {})
                .build(new Path(BLinePathUtil.PATHS_DIR, pathNames[2])),

            // SOTM To Corner
            new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
            new ParallelCommandGroup(
                new HandoffRun(),
                new SpindexerRun(),
                new IntakeAutoDigest().repeatedly()
            )

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