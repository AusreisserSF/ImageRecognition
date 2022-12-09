package org.firstinspires.ftc.teamcode.common;

public class RobotConstants {

    public static final String imageDir = "/images/";
    public static final String logDir = "/logs/";
    public static final String xmlDir = "/xml/";
    public static final String imageFilePrefix = "Image_";

    public enum RunType {
        AUTONOMOUS, TELEOP
    }

    public enum Alliance {
        BLUE, RED, NONE
    }

    //**TODO It's a good idea to put all recognition paths here.
    // Generic recognition paths - independent of a particular game
    public enum RecognitionPath {
        GRAYSCALE, RED_CHANNEL_GRAYSCALE, COLOR
    }

    //**TODO replace with RecognitionResults below
    public enum OpenCVResults {
        OCV_ERROR, RECOGNITION_SUCCESSFUL, RECOGNITION_UNSUCCESSFUL
    }

    public enum RecognitionResults {
        RECOGNITION_INTERNAL_ERROR, RECOGNITION_SUCCESSFUL, RECOGNITION_UNSUCCESSFUL
    }


}
