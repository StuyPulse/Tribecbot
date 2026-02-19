/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class SpindexerInterpolation {
    private static final InterpolatingDoubleTreeMap interpolatingDoubleTreeMap;

    private static final double[][] rpmAndDistance = {
        /* { RPM, Distance (m) } */
        {1,1}
    };

    static {
        interpolatingDoubleTreeMap = new InterpolatingDoubleTreeMap();
        for (double[] data: rpmAndDistance) {
            interpolatingDoubleTreeMap.put(data[1], data[0]);
        }
    }

    public static double getRPM(double distanceInMeters){
        return interpolatingDoubleTreeMap.get(distanceInMeters);
    }
}
