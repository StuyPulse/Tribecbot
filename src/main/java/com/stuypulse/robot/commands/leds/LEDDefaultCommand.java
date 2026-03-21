
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.leds;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.RobotMode;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.handoff.Handoff;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.leds.LEDController;
import com.stuypulse.robot.subsystems.spindexer.Spindexer;
import com.stuypulse.robot.subsystems.superstructure.hood.Hood;
import com.stuypulse.robot.subsystems.superstructure.shooter.Shooter;
import com.stuypulse.robot.subsystems.superstructure.turret.Turret;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.vision.LimelightVision;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;

public class LEDDefaultCommand extends Command{
    private final LEDController leds;
    private final CommandSwerveDrivetrain swerve;
    private final Handoff handoff;
    private final Intake intake;
    private final Spindexer spindexer;
    private final Hood hood;
    private final Shooter shooter;
    private final Turret turret;
    private final LimelightVision vision;

    public LEDDefaultCommand() {
        this.leds = LEDController.getInstance();
        this.swerve = CommandSwerveDrivetrain.getInstance();
        this.handoff = Handoff.getInstance();
        this.intake = Intake.getInstance();
        this.spindexer = Spindexer.getInstance();
        this.hood = Hood.getInstance();
        this.shooter = Shooter.getInstance();
        this.turret = Turret.getInstance();
        this.vision = LimelightVision.getInstance();

        addRequirements(leds);
    }

    @Override
    public void execute() {
        if(Robot.getMode() == RobotMode.DISABLED) {
            if (LimelightVision.getInstance().getMaxTagCount() >= Settings.LED.DESIRED_TAGS_WHEN_DISABLED) {
                leds.applyPattern(Settings.LED.DISABLED_ALIGNED);
            }
            else {
                leds.applyPattern(LEDPattern.kOff);
            }
        }
        else {
            if (swerve.isUnderTrench()) {
                leds.applyPattern(Settings.LED.PASSING_TRENCH);
            }
            else if ()
        }
    }
}
