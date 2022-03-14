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

    public enum OpenCVResults {
        OCV_ERROR, RECOGNITION_SUCCESSFUL, RECOGNITION_UNSUCCESSFUL
    }

}
