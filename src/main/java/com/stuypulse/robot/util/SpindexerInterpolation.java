package com.stuypulse.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class SpindexerInterpolation {
    private static final InterpolatingDoubleTreeMap interpolatingDoubleTreeMap;

    private static final double[][] voltageAndDistance = {
        // { voltage, distance }
        {1,1}
    };

    static {
        interpolatingDoubleTreeMap = new InterpolatingDoubleTreeMap();
        for (double[] data: voltageAndDistance) {
            interpolatingDoubleTreeMap.put(data[1], data[0]);
        }
    }

    public static double getRPM(double distanceInMeters){
        return interpolatingDoubleTreeMap.get(distanceInMeters);
    }
}
