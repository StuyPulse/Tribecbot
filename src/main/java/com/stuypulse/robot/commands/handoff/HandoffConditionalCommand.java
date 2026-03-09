/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.    */
/* Use of this source code is governed by an MIT-style license     */
/* that can be found in the repository LICENSE file.               */
/*******************************************************************/

package com.stuypulse.robot.commands.handoff;

import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

public class HandoffConditionalCommand extends ConditionalCommand {
    public HandoffConditionalCommand() {
        super(
            new HandoffStop(),
            new HandoffRun(),
            () -> {
                CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
                Superstructure superstructure = Superstructure.getInstance();
                return swerve.isBehindTower() || swerve.isBehindHub() || superstructure.isTurretWrapping();
            }
        );
    }
}
