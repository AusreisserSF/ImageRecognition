package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.teamcode.common.RobotConstants;

// Holds the results of image recognition.
// Includes both the angle and distance from the camera to an object
// as well as those from the center of the robot to the object.
public class RealSenseReturn {

    public static final double RECOGNITION_ANGLE_NPOS = -360.0;
    public static final double RECOGNITION_DISTANCE_NPOS = -1.0;

    public final RobotConstants.RecognitionResults recognitionResults;
    public final double angleFromCamera;
    public final double distanceFromCamera;
    public final double angleFromRobotCenter;
    public final double distanceFromRobotCenter;

    // Angles are in degrees, distances are in inches.
    public RealSenseReturn(RobotConstants.RecognitionResults pRecognitionResults,
                           double pAngleFromCamera, double pDistanceFromCamera, double pAngleFromRobotCenter, double pDistanceFromRobotCenter) {
        recognitionResults = pRecognitionResults;
        angleFromCamera = pAngleFromCamera;
        distanceFromCamera = pDistanceFromCamera;
        angleFromRobotCenter = pAngleFromRobotCenter;
        distanceFromRobotCenter = pDistanceFromRobotCenter;
    }

    // Constructor for OpenCV errors such as "no such file".
    public RealSenseReturn(RobotConstants.RecognitionResults pRecognitionResults) {
        recognitionResults = pRecognitionResults;
        angleFromCamera = RECOGNITION_ANGLE_NPOS;
        distanceFromCamera = RECOGNITION_DISTANCE_NPOS;
        angleFromRobotCenter = RECOGNITION_ANGLE_NPOS;
        distanceFromRobotCenter = RECOGNITION_DISTANCE_NPOS;
    }
}