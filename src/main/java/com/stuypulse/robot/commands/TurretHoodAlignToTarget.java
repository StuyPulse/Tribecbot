// package com.stuypulse.robot.commands;

// import com.stuypulse.robot.subsystems.hoodedshooter.HoodedShooter;
// import com.stuypulse.robot.subsystems.hoodedshooter.HoodedShooter.HoodedShooterState;
// import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
// import com.stuypulse.robot.util.hoodedshooter.ShotCalculator;
// import com.stuypulse.robot.util.hoodedshooter.ShotCalculator.AlignAngleSolution;
// import com.stuypulse.robot.Robot;
// import com.stuypulse.robot.constants.Constants;
// import com.stuypulse.robot.constants.Field;
// import com.stuypulse.robot.constants.Settings;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Twist2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command; 

// public class TurretHoodAlignToTarget extends Command{
//     private final HoodedShooter hdsr;
//     private final CommandSwerveDrivetrain swerve;
//     // private final Turret turret;

//     private final Field2d field;

//     private FieldObject2d targetPose2d;
//     private FieldObject2d virtualHubPose2d;
//     private FieldObject2d futureShooterPose;

    
//     public TurretHoodAlignToTarget() {
//         hdsr = HoodedShooter.getInstance();
//         swerve = CommandSwerveDrivetrain.getInstance();

//         field = Field.FIELD2D;
//         virtualHubPose2d = field.getObject("virtualHubPose");
//         targetPose2d = field.getObject("targetPose");
//         futureShooterPose = field.getObject("futureShooterPose");

//         addRequirements(hdsr);
//     }
     
//     @Override
//     public void initialize() {
   
//     }

//     @Override
//     public void execute() {
//         // update targetPose each tick
//         Pose3d targetPose = Pose3d.kZero;
//         if (hdsr.getState() == HoodedShooterState.SHOOT) {
//             targetPose = Field.hubCenter3d;
//         } else if (hdsr.getState() == HoodedShooterState.FERRY){
//             targetPose = new Pose3d(Field.getFerryZonePose(swerve.getPose().getTranslation()));
//         }

//         Pose2d currentPose = swerve.getPose();
        
//         ChassisSpeeds robotRelSpeeds = swerve.getChassisSpeeds();
//         ChassisSpeeds fieldRelSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
//             robotRelSpeeds, 
//             currentPose.getRotation()
//         );
        
//         double axMetersPerSecondSquared = swerve.getPigeon2().getAccelerationX().getValueAsDouble();
//         double ayMetersPerSecondSquared = swerve.getPigeon2().getAccelerationY().getValueAsDouble();
        
//         double shooterRPS = hdsr.getTargetRPM() / 60.0;

//         Pose2d shooterPose = currentPose.plus(Constants.HoodedShooter.TURRET_OFFSET).exp(
//             new Twist2d(
//                 robotRelSpeeds.vxMetersPerSecond * Settings.HoodedShooter.UPDATE_DELAY.doubleValue(),
//                 robotRelSpeeds.vyMetersPerSecond * Settings.HoodedShooter.UPDATE_DELAY.doubleValue(),
//                 0
//             )
//         );
        
//         AlignAngleSolution sol = ShotCalculator.solveShootOnTheFly(
//             new Pose3d(shooterPose.getX(), shooterPose.getY(), Constants.HoodedShooter.TURRET_HEIGHT, Rotation3d.kZero),
//             targetPose,
//             axMetersPerSecondSquared,
//             ayMetersPerSecondSquared,
//             fieldRelSpeeds,
//             shooterRPS, 
//             Constants.Align.MAX_ITERATIONS,
//             Constants.Align.TIME_TOLERANCE
//         );

//         // hdsr.setTargetAngle(sol.launchPitchAngle()); // this doesn't work with the HoodedShooter structure we want
        
//         // this is the required yaw for shooting into the effective hub
//         Rotation2d targetTurretAngle = sol.requiredYaw().minus(currentPose.getRotation());

//         // transform for sim since blue is always origin
//         targetPose2d.setPose(Robot.isBlue() ? targetPose.toPose2d() : Field.transformToOppositeAlliance(targetPose).toPose2d());
//         virtualHubPose2d.setPose((Robot.isBlue() ? sol.estimateTargetPose() : Field.transformToOppositeAlliance(sol.estimateTargetPose())).toPose2d());
//         futureShooterPose.setPose((Robot.isBlue() ? shooterPose : Field.transformToOppositeAlliance(shooterPose)));
  
  

//         SmartDashboard.putNumber("HoodedShooter/calculated yaw", targetTurretAngle.getDegrees());
//         SmartDashboard.putNumber("HoodedShooter/launch pitch angle", sol.launchPitchAngle().getDegrees());
//         // TODO: set turret angle here    
//     }
// }