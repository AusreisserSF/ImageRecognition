package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.teamcode.common.RobotConstants;

// Holds the results of image recognition.
public class RealSenseReturn {

    public static final double RECOGNITION_ANGLE_NPOS = -360.0;
    public static final double RECOGNITION_DISTANCE_NPOS = -1.0;

    public final RobotConstants.RecognitionResults recognitionResults;
    public final double angleFromRobotCenter;
    public final double distanceFromRobotCenter;

    // Parameter pAngle = the angle from the center of the robot to the
    // center of the gold cube. Parameter pDistance = the distance in
    // meters from the center of the robot to the center of the gold cube.
    public RealSenseReturn(RobotConstants.RecognitionResults pRecognitionResults, double pAngle, double pDistance) {
        recognitionResults = pRecognitionResults;
        angleFromRobotCenter = pAngle;
        distanceFromRobotCenter = pDistance;
    }

    // Constructor for OpenCV errors such as "no such file".
    public RealSenseReturn(RobotConstants.RecognitionResults pRecognitionResults) {
        recognitionResults = pRecognitionResults;
        angleFromRobotCenter = RECOGNITION_ANGLE_NPOS;
        distanceFromRobotCenter = RECOGNITION_DISTANCE_NPOS;
    }
}