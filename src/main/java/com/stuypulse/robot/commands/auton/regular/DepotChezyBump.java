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

public class DepotChezyBump extends SequentialCommandGroup {
    public DepotChezyBump(PathPlannerPath... paths) {
        addCommands( 
            
            new SwerveResetPose(paths[0].getStartingHolonomicPose().get()),

            Commands.defer(() -> new WaitCommand(RobotContainer.getWaitTimeOne()), Set.of()),

            new SuperstructureInterpolation(),
            new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
            new WaitCommand(Seconds.of(2)).deadlineFor( //configure for follow delay
                new HandoffRun(),
                new SpindexerRun(),
                new IntakeAutoDigest()
            ).andThen( 
            //extra precaution bcs reviewing the logic, i dont see anything set the state back after the respecive Commands finish. 
            //worked b4 so if this changes the behavior, just remove it
                new HandoffStop(),
                new SpindexerStop()
            ),

            new SuperstructureAutoInterpolation().alongWith(new IntakeDeploy()),

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]),
            
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[3]),

            // new SwerveResetPose(CommandSwerveDrivetrain.getInstance().getPose()), //MT2 fused pose bcs of addVisionmeasurement
            new WaitCommand(Seconds.of(1.3)), //let robot stabilize after crossing bump 
            //if it does not work, remove and increase wait time to 1+ seconds. Will delay but probably be accurate.
            //alt = reset to the paths[3].getHolonomicStartingPose().get(). Assumes bump will always go perfect and only pose drift occurs, less accurate, but faster than waiting

            new SuperstructureSOTM(),
            new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[4]).alongWith(
                new HandoffRun(),
                new SpindexerRun(),
                new IntakeAutoDigest()
                //all get turned off during teleop init - and this also means that after the path finishes, we will keep shooting in auton
            ),
            new IntakeAutoDigest()
        );

    }
}
