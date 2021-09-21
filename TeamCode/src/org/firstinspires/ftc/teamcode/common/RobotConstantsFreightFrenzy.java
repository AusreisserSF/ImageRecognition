package org.firstinspires.ftc.teamcode.common;

public class RobotConstantsFreightFrenzy {

    public enum OpMode {
        RED_CAROUSEL, RED_WAREHOUSE, BLUE_CAROUSEL, BLUE_WAREHOUSE, TEST,
        TELEOP_AUTO_DRIVE // for use in TeleOp only - does not appear on the driver station
    }

    //**TODO These may be relative to the ROI, i.e. LEFT may actually be the center
    // barcode - rename to avoid confusion? BARCODE_0_IN_ROI ...
    public enum BarcodePosition {
        LEFT, CENTER, RIGHT, BARCODE_NPOS
    }

    // Vumark identifiers
    //public enum SupportedVumark {BLUE_TOWER_GOAL, RED_TOWER_GOAL, RED_ALLIANCE, BLUE_ALLIANCE, FRONT_WALL}

}