package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.teamcode.common.RobotConstants;

// Holds the results of image recognition.
public class ConeStackReturn {

    public static final double GOLD_CUBE_ANGLE_NPOS = -360.0;
    public static final double GOLD_CUBE_DISTANCE_NPOS = -1.0;

    public final RobotConstants.OpenCVResults openCVResults;
    public final double angleFromRobotCenter;
    public final double distanceFromRobotCenter;

    // Parameter pAngle = the angle from the center of the robot to the
    // center of the gold cube. Parameter pDistance = the distance in
    // meters from the center of the robot to the center of the gold cube.
    public ConeStackReturn(RobotConstants.OpenCVResults pOpenCVResults, double pAngle, double pDistance) {
        openCVResults = pOpenCVResults;
        angleFromRobotCenter = pAngle;
        distanceFromRobotCenter = pDistance;
    }

    // Constructor for OpenCV errors such as "no such file".
    public ConeStackReturn(RobotConstants.OpenCVResults pOpenCVResults) {
        openCVResults = pOpenCVResults;
        angleFromRobotCenter = GOLD_CUBE_ANGLE_NPOS;
        distanceFromRobotCenter = GOLD_CUBE_DISTANCE_NPOS;
    }
}