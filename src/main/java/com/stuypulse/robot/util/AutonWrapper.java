package com.stuypulse.robot.util;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;

import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonWrapper extends SequentialCommandGroup {
    public List<PathPlannerPath> subPaths = new ArrayList<PathPlannerPath>();

    public AutonWrapper(PathPlannerPath... paths) {
        for (PathPlannerPath path : paths) {
            subPaths.add(path);
        }
    }

    public AutonWrapper() {}

    public void logPaths() {
        for (int i = 0; i < subPaths.size(); i++) {
            if (Robot.isBlue()) {
                Field.FIELD2D.getObject("path: " + subPaths.get(i).name).setPoses(subPaths.get(i).getPathPoses());
            }
            else {
                Field.FIELD2D.getObject("path: " + subPaths.get(i).name).setPoses(Field.transformToOppositeAlliance(subPaths.get(i).getPathPoses()));
            }
        } 
    }

    public void clearFieldObjects() {
        for (int i = 0; i < subPaths.size(); i++) {
            Field.FIELD2D.getObject("path: " + subPaths.get(i).name).setPoses(new ArrayList<>());
            Field.FIELD2D.getObject("path: " + subPaths.get(i).name).close();
        } 
    }
}
