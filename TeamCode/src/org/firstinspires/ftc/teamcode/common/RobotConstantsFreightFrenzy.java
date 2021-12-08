package org.firstinspires.ftc.teamcode.common;

public class RobotConstantsFreightFrenzy {

    public enum OpMode {
        RED_CAROUSEL, RED_WAREHOUSE, BLUE_CAROUSEL, BLUE_WAREHOUSE, TEST,
        TELEOP_AUTO_DRIVE // for use in TeleOp only - does not appear on the driver station
    }

    public enum RecognitionPath {
        GRAY, HSV
    }

    // Relative position of a barcode element within the ROI.
    public enum BarcodeElementWindow {
        LEFT, RIGHT, WINDOW_NPOS
    }

    public enum ShippingHubLevels {
        SHIPPING_HUB_LEVEL_1, SHIPPING_HUB_LEVEL_2, SHIPPING_HUB_LEVEL_3,
        SHIPPING_HUB_LEVEL_NPOS
    }

    // Vumark identifiers
    public enum SupportedVumark {BLUE_ALLIANCE_WALL_TARGET, SHARED_STORAGE_WALL_BLUE_STORAGE_TARGET,
        RED_ALLIANCE_WALL_TARGET, SHARED_STORAGE_WALL_RED_STORAGE_TARGET
    }

}