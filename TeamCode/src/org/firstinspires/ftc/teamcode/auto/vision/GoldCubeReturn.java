package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.teamcode.common.RobotConstants;

// Holds the results of image recognition.
public class GoldCubeReturn {

    public static final double GOLD_CUBE_ANGLE_NPOS = -360.0;
    public static final double GOLD_CUBE_DISTANCE_NPOS = -1.0;

    public final RobotConstants.OpenCVResults openCVResults;
    public final double angleToGoldCube;
    public final double distanceToGoldCube;

    //**TODO angleFromRobotCenter; distanceFromRobotCenter
    public GoldCubeReturn(RobotConstants.OpenCVResults pOpenCVResults, double pAngle, double pDistance) {
        openCVResults = pOpenCVResults;
        angleToGoldCube = pAngle;
        distanceToGoldCube = pDistance;
    }

    // Constructor for OpenCV errors such as "no such file".
    public GoldCubeReturn(RobotConstants.OpenCVResults pOpenCVResults) {
        openCVResults = pOpenCVResults;
        angleToGoldCube = GOLD_CUBE_ANGLE_NPOS;
        distanceToGoldCube = GOLD_CUBE_DISTANCE_NPOS;
    }
}