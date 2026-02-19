package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.commands.swerve.pidToPose.SwerveDrivePIDToPose;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings.Swerve.Alignment.Targets;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SwerveClimbAlignTop extends SwerveDrivePIDToPose{
    public SwerveClimbAlignTop(){
        super(new Pose2d(Field.towerCenter.getX(), Field.towerCenter.getY() + Field.barDisplacement + Field.DISTANCE_TO_RUNGS, new Rotation2d(Units.degreesToRadians(180))));
    }

    @Override
    public void execute(){
        super.execute();
    }
}