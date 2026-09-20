package com.stuypulse.robot.commands.auton.regular;

import static edu.wpi.first.units.Units.Seconds;

import java.util.List;
import java.util.Set;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.handoff.HandoffRun;
import com.stuypulse.robot.commands.intake.IntakeAutoDigest;
import com.stuypulse.robot.commands.intake.IntakeDeploy;
import com.stuypulse.robot.commands.intake.IntakeStow;
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

public class FastFollowRight extends SequentialCommandGroup {
     
    public FastFollowRight(PathPlannerPath... paths) {
        addCommands( 
            new SwerveResetPose(paths[0].getStartingHolonomicPose().get()),

            Commands.defer(() -> new WaitCommand(RobotContainer.getWaitTimeOne()), Set.of()),

            new SuperstructureInterpolation(),
            new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
            new WaitCommand(Seconds.of(1.5)).deadlineFor( //configure for follow delay
                new HandoffRun(),
                new SpindexerRun(),
                new IntakeStow()
            ).andThen( 
            //extra precaution bcs reviewing the logic, i dont see anything set the state back after the respecive Commands finish. 
            //worked b4 so if this changes the behavior, just remove it
                new HandoffStop(),
                new SpindexerStop()
            ),

            new SuperstructureAutoInterpolation().alongWith(new IntakeDeploy()),

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]), // Bump traverse
            new WaitCommand(Seconds.of(0.5)), //let robot stabilize after crossing bump 

            // new SwerveResetPose(CommandSwerveDrivetrain.getInstance().getPose()), //MT2 fused pose bcs of addVisionmeasurement
           
            //if it does not work, remove and increase wait time to 1+ seconds. Will delay but probably be accurate.
            //alt = reset to the paths[3].getHolonomicStartingPose().get(). Assumes bump will always go perfect and only pose drift occurs, less accurate, but faster than waiting

            // Start scoring
            new SuperstructureSOTM(),
            new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
            new HandoffRun(),
            new SpindexerRun(),

            // Depot paths + auto digestion
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[3]).deadlineFor(
                new RepeatCommand(new IntakeAutoDigest())
            ),

            new SuperstructureAutoInterpolation().alongWith(new IntakeDeploy()),

            // NZ Trip 2
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[4]),
                new HandoffStop(),
                new SpindexerStop()
            )
        );
    }
}
