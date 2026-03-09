/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.swerve;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;

import com.stuypulse.robot.constants.DriverConstants.Driver.Drive;
import com.stuypulse.robot.constants.DriverConstants.Driver.Turn;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveDrive extends Command {

    private final CommandSwerveDrivetrain swerve;
    private final Superstructure superstructure;

    private final Gamepad driver;

    private final VStream speed;
    private final IStream turn;

    public SwerveDriveDrive(Gamepad driver) {
        swerve = CommandSwerveDrivetrain.getInstance();
        superstructure = Superstructure.getInstance();

        speed = VStream.create(this::getDriverInputAsVelocity)
        .filtered(
            new VDeadZone(Drive.DEADBAND), 
            x -> x.clamp(1),
            x -> x.pow(Drive.POWER),
            x -> x.mul(Swerve.Constraints.MAX_VELOCITY_M_PER_S),
            new VRateLimit(Swerve.Constraints.MAX_ACCEL_M_PER_S_SQUARED),
            new VLowPassFilter(Drive.RC)
        );

        turn = IStream.create(driver::getRightX)
        .filtered(
            x -> SLMath.deadband(x, Turn.DEADBAND),
            x -> SLMath.spow(x, Turn.POWER),
            x -> x * Swerve.Constraints.MAX_ANGULAR_VEL_RAD_PER_S,
            new LowPassFilter(Turn.RC)
        );

        this.driver = driver;

        addRequirements(swerve);
    }

    private Vector2D getDriverInputAsVelocity() {
        return new Vector2D(driver.getLeftStick().y, -driver.getLeftStick().x);
    }

    @Override
    public void execute() {
        Vector2D speedVector = speed.get();
        double angularVel = turn.get();

        double maxAngularVel = Swerve.Constraints.MAX_ANGULAR_VEL_RAD_PER_S;

        if (speedVector.magnitude() > 0.05 && superstructure.getState() == SuperstructureState.SOTM) {
            speedVector = speedVector.normalize().mul(Settings.Swerve.Constraints.MAX_VELOCITY_SOTM_M_PER_S);
            maxAngularVel = Settings.Swerve.Constraints.MAX_ANGULAR_VEL_SOTM_RAD_PER_S;
        } else if (speedVector.magnitude() > 0.05 && superstructure.getState() == SuperstructureState.FOTM) {
            speedVector = speedVector.normalize().mul(Settings.Swerve.Constraints.MAX_VELOCITY_FOTM_M_PER_S);
            maxAngularVel = Settings.Swerve.Constraints.MAX_ANGULAR_VEL_FOTM_RAD_PER_S;
        }

        angularVel = MathUtil.clamp(angularVel, -maxAngularVel, maxAngularVel);

        swerve.setControl(swerve.getFieldCentricSwerveRequest()
            .withVelocityX(speedVector.x)
            .withVelocityY(speedVector.y)
            .withRotationalRate(-angularVel));
    }
}