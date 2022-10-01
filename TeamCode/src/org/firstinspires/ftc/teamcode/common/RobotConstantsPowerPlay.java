package org.firstinspires.ftc.teamcode.common;

public class RobotConstantsPowerPlay {

    public enum OpMode {
        // Autonomous OpModes
        RED_F2, RED_F5, BLUE_A2, BLUE_A5,
        TEST, TEST_PRE_MATCH, AUTO_NO_DRIVE,

        // TeleOp OpModes
        POWER_PLAY_BLUE, POWER_PLAY_RED, TELEOP_NO_DRIVE,

        // Pseudo OpModes for running Autonomous actions from within
        // TeleOp. These are not "real" OpMoces in that they don't
        // appear on the Driver Station but they are present in
        // RobotAction.xml.
        TELEOP_TAKE_PICTURE
    }

    public enum RecognitionPath {
        GRAY, HSV, REFLECTIVE_TAPE
    }

    // Relative position of a barcode element within the ROI.
    public enum BarcodeElementWindow {
        LEFT, RIGHT, WINDOW_NPOS
    }

    //** 9/25/2022 replace with SignalSleeveLocation
    public enum ShippingHubLevels {
        SHIPPING_HUB_LEVEL_1, SHIPPING_HUB_LEVEL_2, SHIPPING_HUB_LEVEL_3,
        SHIPPING_HUB_LEVEL_NPOS
    }

    public enum SignalSleeveLocation {
        ONE, TWO, THREE, SIGNAL_SLEEVE_LOCATION_NPOS
    }

    // Vumark identifiers
    public enum SupportedVumark {
        VUMARK_1, VUMARK_2, VUMARK_3, VUMARK_4
    }

}