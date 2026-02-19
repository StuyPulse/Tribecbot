package com.stuypulse.robot.constants;

public interface Constants {
    public interface ClimberHopper {
        // TODO: get these limits
        double MIN_HEIGHT_METERS = 0;
        double MAX_HEIGHT_METERS = 10;

        double DRUM_RADIUS_METERS = ((MAX_HEIGHT_METERS - MIN_HEIGHT_METERS) / (Encoders.NUM_ROTATIONS_TO_REACH_TOP / Encoders.GEARING)) / 2 / Math.PI;

        double MASS_KG = 1;

        public interface Encoders {
            // TODO: get these
            double GEARING = 52.0/12.0;

            double NUM_ROTATIONS_TO_REACH_TOP = (MAX_HEIGHT_METERS - MIN_HEIGHT_METERS) / (0.480 / 13); // Number of rotations that the motor has to spin, NOT the gear
            double POSITION_CONVERSION_FACTOR = (MAX_HEIGHT_METERS - MIN_HEIGHT_METERS) / NUM_ROTATIONS_TO_REACH_TOP;
            double VELOCITY_CONVERSION_FACTOR = (MAX_HEIGHT_METERS - MIN_HEIGHT_METERS) / NUM_ROTATIONS_TO_REACH_TOP / 60;
        }
    }
}