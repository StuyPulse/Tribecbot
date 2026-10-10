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

public class RightSimpleTwo extends AutonWrapper {
    public RightSimpleTwo(PathPlannerPath... paths) {
        
        super(paths);

        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

        addCommands(
            new SwerveResetPose(paths[0].getStartingHolonomicPose().get()),
            Commands.defer(() -> new WaitCommand(RobotContainer.getWaitTimeOne()), Set.of()),

            swerve.followPathCommand(paths[0]).alongWith(new WaitCommand(0.2).andThen(new IntakeDeploy())),
            swerve.followPathCommand(paths[1]),
            swerve.followPathCommand(paths[2]).alongWith(new SuperstructureAutoInterpolation()),

            new WaitCommand(0.4),

            new SuperstructureSOTM(),
            new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
            new ParallelCommandGroup(
                swerve.followPathCommand(paths[3]).deadlineFor(new RepeatCommand(new IntakeAutoDigest())),
                new HandoffRun(),
                new SpindexerRun()
            ),
            new SuperstructureAutoInterpolation().alongWith(new IntakeDeploy()),

            new ParallelCommandGroup(
                swerve.followPathCommand(paths[4]),
                new HandoffStop(),
                new SpindexerStop()
            ),
            
            swerve.followPathCommand(paths[5]),
            swerve.followPathCommand(paths[6]),

            new WaitCommand(0.4),

            new SuperstructureSOTM(),
            new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
                //.deadlineFor(swerve.run(() -> swerve.setControl(new SwerveRequest.Idle()))),
            new ParallelCommandGroup(
                swerve.followPathCommand(paths[7]).deadlineFor(new RepeatCommand(new IntakeAutoDigest())),
                new HandoffRun(),
                new SpindexerRun()
            ),
            new SuperstructureAutoInterpolation().alongWith(new IntakeDeploy())
        );
    }
}
