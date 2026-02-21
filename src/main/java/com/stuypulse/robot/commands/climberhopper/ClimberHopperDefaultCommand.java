/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.climberhopper;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.climberhopper.ClimberHopper;
import com.stuypulse.robot.subsystems.climberhopper.ClimberHopper.ClimberHopperState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class ClimberHopperDefaultCommand extends Command {
    private final ClimberHopper climberHopper;
    // private final CommandSwerveDrivetrain swerve;
    // private Pose2d pose;
    // private boolean flag = false; // To prevent repeated stalling under trench

    public ClimberHopperDefaultCommand() {
        climberHopper = ClimberHopper.getInstance();
        // swerve = CommandSwerveDrivetrain.getInstance();
        // pose = swerve.getPose();

        
        addRequirements(climberHopper);
    }

    @Override
    public void execute() {
        // === Robot Position Logic ===
        // Reminder from driver's perspective, positive X to the opposite alliance and positive Y points to the left.
        // pose = swerve.getPose();

        // boolean isBetweenRightTrenchesY = Field.NearRightTrench.rightEdge.getY() < pose.getY() && Field.NearRightTrench.leftEdge.getY() > pose.getY();

        // boolean isBetweenLeftTrenchesY = Field.NearLeftTrench.rightEdge.getY() < pose.getY() && Field.NearLeftTrench.leftEdge.getY() > pose.getY();

        // boolean isCloseToNearTrenchesX = Math.abs(pose.getX() - Field.NearRightTrench.rightEdge.getX()) < Field.trenchXTolerance;

        // boolean isCloseToFarTrenchesX = Math.abs(pose.getX() - Field.FarRightTrench.rightEdge.getX()) < Field.trenchXTolerance;

        // boolean isUnderTrench = (isBetweenRightTrenchesY || isBetweenLeftTrenchesY) && (isCloseToNearTrenchesX || isCloseToFarTrenchesX); // Far X tolerance
        
        // // === Climber Position and State Logic ===
        // boolean inDownState = climberHopper.getState() == ClimberHopperState.CLIMBER_DOWN || climberHopper.getState() == ClimberHopperState.HOPPER_DOWN;

        // boolean stalledByBalls = climberHopper.getStalling() && inDownState;
        // // boolean stalledByBalls = true;

        // if (isUnderTrench) {
        //     if (!stalledByBalls && !flag) {
        //         climberHopper.setState(ClimberHopperState.HOPPER_DOWN);
        //     } else {
        //         climberHopper.setState(ClimberHopperState.HOPPER_UP);
        //         // TODO: Flash LEDs or have Controller buzz when this happens. Also we need to somehow log this state!!!
        //         flag = true; // prevent hopper from going back down while still under trench with too many balls
        //     }
        // } else { // If not under trench...
        //     if (climberHopper.getState() != ClimberHopperState.CLIMBER_DOWN) {
        //         // Set the hopper up
        //         climberHopper.setState(ClimberHopperState.HOPPER_UP);
        //     }
        // }

        // if (!isUnderTrench) {
        //     flag = false;
        // }

        // SmartDashboard.putBoolean("ClimberHopper/UnderTrench", isUnderTrench);

        // !!! AFTER ABANDONING VERTICAL EXPANSION:
        if (climberHopper.getState() != ClimberHopperState.CLIMBER_DOWN) {
            // Set the hopper down
            climberHopper.setState(ClimberHopperState.HOPPER_DOWN);
        }
    }
}