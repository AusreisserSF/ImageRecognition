package org.firstinspires.ftc.teamcode.common;

public class RobotConstantsFreightFrenzy {

    public enum OpMode {
        RED_CAROUSEL, RED_WAREHOUSE, BLUE_CAROUSEL, BLUE_WAREHOUSE, TEST,
        TELEOP_AUTO_DRIVE // for use in TeleOp only - does not appear on the driver station
    }

    // Relative position of a barcode element within the ROI.
    public enum BarcodeElementWithinROI {
        LEFT_WITHIN_ROI, RIGHT_WITHIN_ROI, BARCODE_ELEMENT_NPOS
    }

    public enum ShippingHubLevels {
        SHIPPING_HUB_LEVEL_1, SHIPPING_HUB_LEVEL_2, SHIPPING_HUB_LEVEL_3,
        SHIPPING_HUB_LEVEL_NPOS
    }

    // Vumark identifiers
    //public enum SupportedVumark {BLUE_TOWER_GOAL, RED_TOWER_GOAL, RED_ALLIANCE, BLUE_ALLIANCE, FRONT_WALL}

}