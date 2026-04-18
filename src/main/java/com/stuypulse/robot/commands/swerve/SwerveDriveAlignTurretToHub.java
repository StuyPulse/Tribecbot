/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.swerve;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Gains.Swerve.Alignment;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.superstructure.turret.Turret;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.superstructure.TurretAngleCalculator;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveAlignTurretToHub extends Command {

    private final CommandSwerveDrivetrain swerve;
    private final Turret turret;
    private final AngleController angleController;

    private Pose2d robot;

    public SwerveDriveAlignTurretToHub() {
        swerve = CommandSwerveDrivetrain.getInstance();
        turret = Turret.getInstance();

        angleController = new AnglePIDController(Alignment.THETA.kP, Alignment.THETA.kI, Alignment.THETA.kD)
            .setSetpointFilter(new AMotionProfile(Settings.Swerve.Constraints.MAX_ANGULAR_VEL_RAD_PER_S, Settings.Swerve.Constraints.MAX_ANGULAR_ACCEL_RAD_PER_S_SQUARED));
                
        addRequirements(swerve);
    }

    protected double getAngleError() {
        return angleController.getError().getRotation2d().getDegrees();
    }

    public Rotation2d getTargetAngle() {
        Translation2d robot = swerve.getPose().getTranslation();
        Translation2d hub = Field.getHubPose().getTranslation();
        return robot.minus(hub).getAngle().plus(Rotation2d.k180deg);
    }

    @Override
    public void execute() {
        robot = swerve.getPose();

        swerve.setControl(swerve.getFieldCentricSwerveRequest()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(angleController.update(
                Angle.fromRotation2d(getTargetAngle()),
                Angle.fromRotation2d(robot.getRotation()))));

        DogLog.log("Swerve/Angle Error", angleController.getError().toDegrees());
        DogLog.log("Swerve/Target Angle Hub Deg", TurretAngleCalculator.getPointAtTargetAngle(Field.getHubPose().getTranslation(), swerve.getTurretPose().getTranslation(), robot.getRotation()).getDegrees());
    }

    @Override
    public boolean isFinished() {
        return angleController.isDoneDegrees(Settings.Swerve.Alignment.Tolerances.THETA_TOLERANCE_DEG);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(new SwerveRequest.SwerveDriveBrake());
    }
}