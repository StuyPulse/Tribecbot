/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.util.turret;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.Rotation2d;

public class TurretAngleCalculator {

    private static final double MAX_ANGLE_DEGREES = Settings.Turret.MAX_THEORETICAL_ROTATION.getDegrees();
    private static final double MIN_ANGLE_DEGREES = Settings.Turret.MIN_THEORETICAL_ROTATION.getDegrees();
    private static final double RESOLUTION = Settings.Turret.RESOLUTION_OF_ABSOLUTE_ENCODER;
    private static final int NUM_POINTS = (int) ((MAX_ANGLE_DEGREES - MIN_ANGLE_DEGREES) / RESOLUTION);
    private static int leastDistanceIndex = 0;

    private static final double[] mechanismAngles = new double[NUM_POINTS];
    private static final double[] ARRAY_17T = generateEncoderValues(17);
    private static final double[] ARRAY_18T = generateEncoderValues(18);

    private static double[] generateEncoderValues(int teeth) {
        double[] values = new double[NUM_POINTS];
        double gearRatio = 1.0 * Settings.Turret.Constants.BigGear.TEETH / teeth;
        int i = 0;

        for (double angle = MIN_ANGLE_DEGREES; angle < MAX_ANGLE_DEGREES; angle += RESOLUTION) {
            mechanismAngles[i] = angle;
            values[i] = (((angle * gearRatio) % 360.0) + 360.0) % 360.0;
            i++;
        }

        return values;
    }

    public static Rotation2d getAbsoluteAngle(double encoder17TValue, double encoder18TValue) {
        double leastDistance = Double.MAX_VALUE;
        for (int i = 0; i < NUM_POINTS; i++) {
            double diff17 = Math.abs(encoder17TValue - ARRAY_17T[i]);
            double diff18 = Math.abs(encoder18TValue - ARRAY_18T[i]);

            diff17 = Math.min(diff17, 360.0 - diff17);
            diff18 = Math.min(diff18, 360.0 - diff18);

            double distance17 = diff17 * diff17;
            double distance18 = diff18 * diff18;

            double distance = distance17 + distance18;

            if (distance < leastDistance) {
                leastDistance = distance;
                leastDistanceIndex = i;
            }
        }

        return Rotation2d.fromDegrees(mechanismAngles[leastDistanceIndex]);
    }

    public static int lowestDistanceIndex() {
        return leastDistanceIndex;
    }
}
