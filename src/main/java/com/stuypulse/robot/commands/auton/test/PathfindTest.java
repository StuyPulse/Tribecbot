package com.stuypulse.robot.commands.auton.test;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.swerve.SwerveResetPose;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class PathfindTest extends SequentialCommandGroup {
    public PathfindTest(PathPlannerPath... paths) {
        addCommands(
            new SwerveResetPose(paths[0].getStartingHolonomicPose().get()),
            
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
            new WaitCommand(10),
            CommandSwerveDrivetrain.getInstance().pathfindThenFollowPath(paths[1], PathConstraints.unlimitedConstraints(12.0))
        );
    }
}
