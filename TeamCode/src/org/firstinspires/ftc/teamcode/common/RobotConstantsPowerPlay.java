package org.firstinspires.ftc.teamcode.common;

public class RobotConstantsPowerPlay {

    public static final float WIDTH_OF_CONE_STACK = 4.0f; // inches
    public static final float WIDTH_OF_JUNCTION = 1.0f; // inches
    public static final float WIDTH_OF_GOLD_CUBE = 2.0f; // inches
    
    public enum OpMode {
        // Autonomous OpModes
        RED_F2, RED_F5, BLUE_A2, BLUE_A5,
        TEST, TEST_PRE_MATCH, AUTO_NO_DRIVE,

        // TeleOp OpModes
        POWER_PLAY, TELEOP_NO_DRIVE,

        // Pseudo OpModes for running Autonomous actions from within
        // TeleOp. These are not "real" OpMoces in that they don't
        // appear on the Driver Station but they are present in
        // RobotAction.xml.
        TELEOP_TAKE_PICTURE_D405
    }

    public enum CameraImageSource {
        VUFORIA, D405_SLEEVE, D405_CONES, D405_JUNCTION
    }

    public enum D405CameraId {
        SWIVEL, SIGNAL_SLEEVE
    }
 
    public enum SignalSleeveRecognitionPath {
        COLOR, GRAYSCALE_SLASH
    }

    public enum AngleAlgorithm {
        REALSENSE, FIELD_OF_VIEW
    }

    public enum SignalSleeveLocation {
        LOCATION_1, LOCATION_2, LOCATION_3, SIGNAL_SLEEVE_LOCATION_NPOS
    }

    public enum SwivelCameraDirection {
        NEUTRAL, CONES, JUNCTION
    }

    public enum ConeStackRecognitionPath {
        RED_CHANNEL_GRAYSCALE, BLUE_CHANNEL_GRAYSCALE, COLOR
    }

    public enum JunctionRecognitionPath {
        GRAYSCALE, TWO_CHANNEL_GRAYSCALE, COLOR
    }

    // Vumark identifiers
    public enum SupportedVumark {
        VUMARK_1, VUMARK_2, VUMARK_3, VUMARK_4
    }

}