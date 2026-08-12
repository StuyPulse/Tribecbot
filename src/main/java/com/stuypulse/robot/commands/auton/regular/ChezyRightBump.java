package com.stuypulse.robot.commands.auton.regular;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Set;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.handoff.HandoffRun;
import com.stuypulse.robot.commands.intake.IntakeAutoDigest;
import com.stuypulse.robot.commands.intake.IntakeDeploy;
import com.stuypulse.robot.commands.spindexer.SpindexerRun;
import com.stuypulse.robot.commands.superstructure.SuperstructureAutoInterpolation;
import com.stuypulse.robot.commands.superstructure.SuperstructureInterpolation;
import com.stuypulse.robot.commands.superstructure.SuperstructureSOTM;
import com.stuypulse.robot.commands.swerve.SwerveResetPose;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import com.stuypulse.robot.commands.handoff.HandoffStop;
import com.stuypulse.robot.commands.spindexer.SpindexerStop;

import edu.wpi.first.wpilibj2.command.*;

public class ChezyRightBump extends SequentialCommandGroup {
    public ChezyRightBump(PathPlannerPath... paths) {
        addCommands( 
            
            new SwerveResetPose(paths[0].getStartingHolonomicPose().get()),

            Commands.defer(() -> new WaitCommand(RobotContainer.getWaitTimeOne()), Set.of()),

            new SuperstructureInterpolation(),
            new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
            new WaitCommand(Seconds.of(2)).deadlineFor(
                new HandoffRun(),
                new SpindexerRun(),
                new IntakeAutoDigest()
            ).andThen(
                new HandoffStop(),
                new SpindexerStop()
            ),

            new SuperstructureAutoInterpolation().alongWith(new IntakeDeploy()),

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),

            new WaitCommand(Seconds.of(1)),

            new SuperstructureSOTM(),
            new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]).alongWith(
                new HandoffRun(),
                new SpindexerRun(),
                new IntakeAutoDigest()
            )


        );

    }
}
