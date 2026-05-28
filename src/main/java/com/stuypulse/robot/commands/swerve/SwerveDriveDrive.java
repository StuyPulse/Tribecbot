/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.swerve;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.stuypulse.robot.constants.DriverConstants.Driver.Drive;
import com.stuypulse.robot.constants.DriverConstants.Driver.Turn;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveDrive extends Command {

    private final CommandSwerveDrivetrain swerve;

    private final Gamepad driver;

    private final VStream speed;
    private final IStream turn;

    private final BStream isIdle;
    private boolean isIdleInit;

    public SwerveDriveDrive(Gamepad driver) {
        swerve = CommandSwerveDrivetrain.getInstance();

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
        isIdle = BStream.create(
            () -> getDriverInputAsVelocity().magnitude() <= Drive.DEADBAND && Math.abs(driver.getRightX()) <= Turn.DEADBAND)
                .filtered(new BDebounce.Rising(0.5), new BDebounce.Falling(0.1));
        isIdleInit = false;
        addRequirements(swerve);
    }

    private Vector2D getDriverInputAsVelocity() {
        return new Vector2D(driver.getLeftStick().y, -driver.getLeftStick().x);
    }

    @Override
    public void execute() {
        if (isIdle.get()) {
            // if (!isIdleInit) {
            //     CommandScheduler.getInstance().schedule(new IntakeAutoDigest().repeatedly().onlyWhile(() -> isIdle.get()).andThen(new IntakeDeploy()));
                swerve.setControl(new SwerveRequest.SwerveDriveBrake());
            // }
            isIdleInit = true;
        } else { 
            swerve.setControl(swerve.getFieldCentricSwerveRequest()
                .withVelocityX(speed.get().x)
                .withVelocityY(speed.get().y)
                .withRotationalRate(-turn.getAsDouble())
            );
            isIdleInit = false;
        }
    }
}