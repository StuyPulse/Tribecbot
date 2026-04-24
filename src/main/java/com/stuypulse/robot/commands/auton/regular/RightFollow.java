package com.stuypulse.robot.commands.auton.regular;

import java.util.Set;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.handoff.HandoffRun;
import com.stuypulse.robot.commands.handoff.HandoffStop;
import com.stuypulse.robot.commands.intake.IntakeAutoDigest;
import com.stuypulse.robot.commands.intake.IntakeDeploy;
import com.stuypulse.robot.commands.spindexer.SpindexerRun;
import com.stuypulse.robot.commands.spindexer.SpindexerStop;
import com.stuypulse.robot.commands.superstructure.SuperstructureAutoInterpolation;
import com.stuypulse.robot.commands.superstructure.SuperstructureSOTM;
import com.stuypulse.robot.commands.swerve.SwerveResetPose;
import com.stuypulse.robot.constants.Gains.Spindexer;
import com.stuypulse.robot.subsystems.handoff.Handoff;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class RightFollow extends SequentialCommandGroup {
    
    public RightFollow(PathPlannerPath... paths) {

        addCommands(
            
            new SwerveResetPose(paths[0].getStartingHolonomicPose().get()),

            // Preloads
            new SuperstructureSOTM(),
            new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
            new ParallelCommandGroup(
                new HandoffRun(),
                new SpindexerRun(),
                Commands.defer(() -> new WaitCommand(RobotContainer.getWaitTimeOne() + 1.0), Set.of()),
                new WaitCommand(1.0).andThen(new IntakeDeploy())
            ),

            // To NZ
            new ParallelCommandGroup(
                new HandoffStop(),
                new SpindexerStop(),
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0])
            ),

            Commands.defer(() -> new WaitCommand(RobotContainer.getWaitTimeTwo()), Set.of()),

            // SOTM To Corner
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),
                new WaitCommand(3.0).andThen(
                    new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance())
                        .andThen(
                            new ParallelCommandGroup(
                                new HandoffRun(),
                                new SpindexerRun()))
                )
            ),

            new IntakeAutoDigest().repeatedly()

        );

    }

}
