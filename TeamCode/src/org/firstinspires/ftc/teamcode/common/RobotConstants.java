package org.firstinspires.ftc.teamcode.common;

public class RobotConstants {

    public static final String imageDir = "/images/";
    public static final String logDir = "/logs/";
    public static final String xmlDir = "/xml/";
    public static final String imageFilePrefix = "Image_";

    //**TODO Move to D405Config.xml and a separate file D405Config.java
    public static final double D405_FOV = 87.0;
    public static final float D405_DEPTH_SCALE = .0001f;
    public static final double D405_CAMERA_TO_ROBOT_CENTER_METERS = 0.20; // 2.5" = 0.0635 meters

    public enum RunType {
        AUTONOMOUS, TELEOP
    }

    public enum Alliance {
        BLUE, RED, NONE
    }

    //**TODO replace with RecognitionResults below
    public enum OpenCVResults {
        OCV_ERROR, RECOGNITION_SUCCESSFUL, RECOGNITION_UNSUCCESSFUL
    }

    public enum RecognitionResults {
        RECOGNITION_INTERNAL_ERROR, RECOGNITION_SUCCESSFUL, RECOGNITION_UNSUCCESSFUL
    }


}
