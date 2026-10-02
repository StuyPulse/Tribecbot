package com.stuypulse.robot.commands.auton.regular;

import java.util.Set;

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
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.AutonWrapper;

import edu.wpi.first.wpilibj2.command.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;

public class LeftSimpleTwo extends AutonWrapper {
    public LeftSimpleTwo(PathPlannerPath... paths) {
        
        super(paths);

        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

        addCommands(
            new SwerveResetPose(paths[0].getStartingHolonomicPose().get()),
            Commands.defer(() -> new WaitCommand(RobotContainer.getWaitTimeOne()), Set.of()),

            swerve.followPathCommand(paths[0]).alongWith(new WaitCommand(0.2).andThen(new IntakeDeploy())),
            swerve.followPathCommand(paths[1]).alongWith(new SuperstructureAutoInterpolation()),

            new SuperstructureSOTM(),
            new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
                //.deadlineFor(swerve.run(() -> swerve.setControl(new SwerveRequest.Idle()))),
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]),
                new HandoffRun(),
                new SpindexerRun(),
                new WaitCommand(0.5)
                    .andThen(new IntakeAutoDigest().until(() -> Superstructure.getInstance().isHopperEmpty()).withTimeout(1))
            ),
            new SuperstructureAutoInterpolation().alongWith(new IntakeDeploy()),

            new ParallelCommandGroup(
                swerve.followPathCommand(paths[3]),
                new HandoffStop(),
                new SpindexerStop()
            ),  

            new SuperstructureSOTM(),
            new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
                //.deadlineFor(swerve.run(() -> swerve.setControl(new SwerveRequest.Idle()))),
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[4]),
                new HandoffRun(),
                new SpindexerRun(),
                new WaitCommand(0.5)
                    .andThen(new IntakeAutoDigest().until(() -> Superstructure.getInstance().isHopperEmpty()).withTimeout(5.0))
            ),
            new SuperstructureAutoInterpolation().alongWith(new IntakeDeploy()),

            new ParallelCommandGroup(
                swerve.followPathCommand(paths[5]),
                new HandoffStop(),
                new SpindexerStop()
            )
        );
    }
}
