// package com.stuypulse.robot.util;

// import com.stuypulse.robot.constants.Settings;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.units.Unit;
// import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj.util.Color8Bit;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

// public class IntakeVisualizer {

//     public static IntakeVisualizer instance;

//     static {
//         instance = new IntakeVisualizer();
//     }

//     public static IntakeVisualizer getInstance() {
//         return instance;
//     }

//     private final Mechanism2d canvas;
//     private double width, height;

//     private final MechanismRoot2d intakePivot;
//     private final MechanismRoot2d leftPivot;
//     private final MechanismLigament2d intakeBar1;
//     private final MechanismLigament2d intakeBar2;
//     private final MechanismLigament2d intakeBar3;
//     private final MechanismLigament2d intakeBar4;
//     private final MechanismLigament2d intakeBar5;
//     private final MechanismLigament2d intakeBar6;

//     private final MechanismLigament2d[] compliantLigaments;
//     private final MechanismLigament2d[] polyLigaments1;
//     private final MechanismLigament2d[] polyLigaments2;

//     private final MechanismLigament2d[] intakeBars;

//     private IntakeVisualizer() {
//         width = Units.inchesToMeters(35);
//         height = Units.inchesToMeters(35);

//         canvas = new Mechanism2d(width, height);
        
//         intakePivot = canvas.getRoot("Intake Pivot", width/2, height/2);
    
//         leftPivot = canvas.getRoot("Left Pivot", width/2 - Units.inchesToMeters(6), height/2);

//         intakeBar1 = new MechanismLigament2d(
//             "Intake Bar 1",
//             Units.inchesToMeters(12.5),
//             180,
//             4,
//             new Color8Bit(Color.kRed));

//         intakeBar2 = new MechanismLigament2d(
//             "Intake Bar 2",
//             Units.inchesToMeters(4),
//             90,
//             4,
//             new Color8Bit(Color.kRed));

//         intakeBar3 = new MechanismLigament2d(
//             "Intake Bar 3",
//             Units.inchesToMeters(6),
//             30,
//             4,
//             new Color8Bit(Color.kRed));

//         intakeBar4 = new MechanismLigament2d(
//             "Intake Bar 4",
//             Units.inchesToMeters(6),
//             40,
//             4,
//             new Color8Bit(Color.kRed));

//         intakeBar5 = new MechanismLigament2d(
//             "Intake Bar 5",
//             Units.inchesToMeters(8),
//             87,
//             4,
//             new Color8Bit(Color.kRed));

//         intakeBar6 = new MechanismLigament2d(
//             "Intake Bar 6",
//             Units.inchesToMeters(2),
//             28,
//             4,
//             new Color8Bit(Color.kRed));

//         compliantLigaments = new MechanismLigament2d[4];
//         for(int i = 0; i < 4; i++){
//             compliantLigaments[i] = new MechanismLigament2d(
//                 "Compliant Ligaments " + (i+1), 
//                 Units.inchesToMeters(2), 
//                 90 * i,
//                 2,
//                 new Color8Bit(Color.kSkyBlue)
//             );
//             intakeBar6.append(compliantLigaments[i]);
//         }

//         polyLigaments1 = new MechanismLigament2d[4];
//         for(int i = 0; i < 4; i++){
//             polyLigaments1[i] = new MechanismLigament2d(
//                 "Polycarb Ligaments " + (i+1), 
//                 Units.inchesToMeters(2), 
//                 90 * i,
//                 2,
//                 new Color8Bit(Color.kBlueViolet)
//             );
//             intakeBar2.append(polyLigaments1[i]);
//         }

//         polyLigaments2 = new MechanismLigament2d[4];
//         for(int i = 0; i < 4; i++){
//             polyLigaments2[i] = new MechanismLigament2d(
//                 "Polycarb Ligaments " + (i+1), 
//                 Units.inchesToMeters(2), 
//                 90 * i,
//                 2,
//                 new Color8Bit(Color.kBlue)
//             );
//             intakeBar1.append(polyLigaments2[i]);
//         }

//         intakePivot.append(intakeBar1);
//         intakeBar1.append(intakeBar2);
//         intakeBar2.append(intakeBar3);
//         intakeBar3.append(intakeBar4);

//         leftPivot.append(intakeBar5);
//         intakeBar5.append(intakeBar6);

//         intakeBars = new MechanismLigament2d[6];
//         intakeBars[0] = intakeBar1;
//         intakeBars[1] = intakeBar2;
//         intakeBars[2] = intakeBar3;
//         intakeBars[3] = intakeBar4;
//         intakeBars[4] = intakeBar5;
//         intakeBars[5] = intakeBar6;
//     }
    
//     public void updateRollers(double speed) {
//         for(int i = 0; i < 4; i++) {
//             compliantLigaments[i].setAngle(compliantLigaments[i].getAngle() + (speed) * 20);
//             polyLigaments1[i].setAngle(polyLigaments1[i].getAngle() + (speed) * 20);
//             polyLigaments2[i].setAngle(polyLigaments2[i].getAngle() + (speed) * 20);
//         }
//     }
//     private void calculate6BarAngles(Rotation2d intakeAngle) {
//         double theta1 = intakeAngle.getRadians();
//         intakeBar1.setAngle(intakeAngle.getDegrees());

//         double theta2 = theta1 * 0.7 - Math.toRadians(20);
//         intakeBar2.setAngle(Math.toDegrees(theta2));

//         double theta3 = -theta1 * 0.5 + Math.toRadians(30);
//         intakeBar3.setAngle(Math.toDegrees(theta3));

//         double theta4 = theta3 + Math.toRadians(90);
//         intakeBar4.setAngle(Math.toDegrees(theta4));

//         double theta5 = theta1 * 0.8 + Math.toRadians(10);
//         intakeBar5.setAngle(Math.toDegrees(theta5));

//         double theta6 = theta2 + Math.toRadians(15);
//         intakeBar6.setAngle(Math.toDegrees(theta6));
//     }

//     public void updateIntakeStuff(Rotation2d intakeAngle, double rollerSpeed, boolean atTargetPosition) {
//         intakePivot.setPosition(width/2, height/2);
//         leftPivot.setPosition(width/2 - Units.inchesToMeters(6), height/2);

//         intakeBar1.setAngle(intakeAngle);

//         for (MechanismLigament2d lig : intakeBars) {
//             lig.setColor(atTargetPosition ? new Color8Bit(Color.kGreen) : new Color8Bit(Color.kRed));
//         }

//         updateRollers(rollerSpeed);
//         calculate6BarAngles(intakeAngle);
        
//         SmartDashboard.putData("Visualizers/Intake", canvas);
//     }
// }
