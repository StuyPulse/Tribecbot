/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.util.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class VisualizerHood {

    private static final VisualizerHood instance;

    static {
        instance = new VisualizerHood();
    }

    public static VisualizerHood getInstance() {
        return instance;
    }

    private final Mechanism2d canvas;
    private final double width, height;

    private final MechanismRoot2d hoodPivot;

    private final MechanismLigament2d hoodArm;

    private final MechanismLigament2d hoodTip;

    private VisualizerHood() {
        width  = Units.inchesToMeters(30);
        height = Units.inchesToMeters(30);

        canvas = new Mechanism2d(width, height);

        hoodPivot = canvas.getRoot("Hood Pivot", width / 2, height * 0.2);

        hoodArm = new MechanismLigament2d(
            "Hood Arm",
            Units.inchesToMeters(12),
            0.0,
            6,
            new Color8Bit(Color.kOrange)
        );

        hoodTip = new MechanismLigament2d(
            "Hood Tip",
            Units.inchesToMeters(3),
            -90.0,
            4,
            new Color8Bit(Color.kOrangeRed)
        );

        hoodPivot.append(hoodArm);
        hoodArm.append(hoodTip);
    }

    public void update(Rotation2d hoodAngle, boolean atTarget) {
        hoodArm.setAngle(hoodAngle);
        hoodArm.setColor(atTarget ? new Color8Bit(Color.kGreen) : new Color8Bit(Color.kRed));
        hoodTip.setColor(atTarget ? new Color8Bit(Color.kGreen) : new Color8Bit(Color.kRed));

        SmartDashboard.putData("Visualizers/Hood", canvas);
    }
}