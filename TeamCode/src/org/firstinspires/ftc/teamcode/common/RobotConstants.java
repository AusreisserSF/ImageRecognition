package org.firstinspires.ftc.teamcode.common;

public class RobotConstants {

    public static final String imageDir = "/images/";
    public static final String logDir = "/logs/";
    public static final String xmlDir = "/xml/";

    public enum RunType {
        AUTONOMOUS, TELEOP
    }

    public enum Alliance {
        BLUE, RED, UNKNOWN
    }

    public enum OpenCVResults {
        INTERNAL_ERROR, RECOGNITION_SUCCESSFUL, RECOGNITION_UNSUCCESSFUL
    }

}
