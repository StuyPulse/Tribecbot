
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
import com.stuypulse.robot.subsystems.handoff.Handoff.HandoffState;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.intake.Intake.PivotState;
import com.stuypulse.robot.subsystems.intake.Intake.RollerState;
import com.stuypulse.robot.subsystems.leds.LEDController;
import com.stuypulse.robot.subsystems.spindexer.Spindexer;
import com.stuypulse.robot.subsystems.spindexer.Spindexer.SpindexerState;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import com.stuypulse.robot.subsystems.superstructure.hood.Hood;
import com.stuypulse.robot.subsystems.superstructure.shooter.Shooter;
import com.stuypulse.robot.subsystems.superstructure.turret.Turret;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.vision.LimelightVision;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
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
    private final Superstructure superstructure;
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
        this.superstructure = Superstructure.getInstance();
        this.vision = LimelightVision.getInstance();

        addRequirements(leds);
    }

    @Override
    public void execute() {
        String state = "NONE";
        if (Robot.getMode() == RobotMode.DISABLED) {
            if (LimelightVision.getInstance().getMaxTagCount() >= Settings.LED.DESIRED_TAGS_WHEN_DISABLED) {
                leds.applyPattern(Settings.LED.DISABLED_ALIGNED);
                state = "DISABLED_ALLOWED";
            }
            else {
                leds.applyPattern(LEDPattern.solid(Color.kRed));
                state = "DISABLED_DISALLOWED";
            }
        }

        else {
            if (swerve.isUnderTrench()) {
                leds.applyPattern(Settings.LED.PASSING_TRENCH);
                state = "UNDER_TRENCH";
            }
            else if (turret.isWrapping()) {
                leds.applyPattern(Settings.LED.TURRET_WRAPPING);
                state = "TURRET_WRAPPING";
            }
            else if (superstructure.getState() == SuperstructureState.LEFT_CORNER) {
                leds.applyPattern(Settings.LED.LEFT_CORNER);
                state = "LEFT_CORNER";
            }
            else if (superstructure.getState() == SuperstructureState.RIGHT_CORNER) {
                leds.applyPattern(Settings.LED.RIGHT_CORNER);
                state = "RIGHT_CORNER";
            } 
            else if (superstructure.getState() == SuperstructureState.KB) {
                leds.applyPattern(Settings.LED.KB_DISTANCE);
                state = "KITBOT";
            }
            else if (superstructure.getState() == SuperstructureState.SOTM) {
                leds.applyPattern(Settings.LED.SOTM_ON);
                state = "SOTM";
            }
            else if (superstructure.getState() == SuperstructureState.FOTM) {
                leds.applyPattern(Settings.LED.FOTM_ON);
                state = "FOTM";
            }
            else if (spindexer.getState() == SpindexerState.REVERSE || 
                     handoff.getState() == HandoffState.REVERSE ||
                     intake.getRollerState() == RollerState.OUTTAKE) {
                leds.applyPattern(Settings.LED.REVERSE);
                state = "REVERSE";
            } 
            else if (intake.getPivotState() == PivotState.STOW) {
                leds.applyPattern(Settings.LED.INTAKE_STOW);
                state = "INTAKE_STOW";
            }
            else if (intake.getPivotState() == PivotState.DEPLOY) {
                leds.applyPattern(Settings.LED.INTAKE_DEPLOYED);
                state = "INTAKE_DEPLOYED";
            }
            else {
                leds.applyPattern(LEDPattern.solid(Color.kRed));
            }
        }

        SmartDashboard.putString("Leds/State", state);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}