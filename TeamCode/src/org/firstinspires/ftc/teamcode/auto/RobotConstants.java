package org.firstinspires.ftc.teamcode.auto;

public class RobotConstants {

    public static final String ringImageDir = "/images/";

    public enum EncoderId {
        LEFT_FRONT_DRIVE_ENCODER, RIGHT_FRONT_DRIVE_ENCODER,
        LEFT_BACK_DRIVE_ENCODER, RIGHT_BACK_DRIVE_ENCODER
    }

    public enum StrafeDirection {
        LEFT, RIGHT
    }

    public enum TurnType {
        NORMALIZED, UNNORMALIZED
    }

    public static final double COUNTS_PER_MOTOR_REV = 537.6; // Neverest40 1120
    public static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    public static final double WHEEL_DIAMETER_INCHES = 4.0;
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These power values must never be negative!!
    public static final double DRIVE_POWER = 0.5; // default power for straight runs and strafing
    public static final double MINIMUM_DRIVE_POWER = 0.2; // 0.3; // minimum power to turn the wheels
    public static final double RAMP_DOWN_DISTANCE = 24.0; // inches
    public static final double RAMP_DOWN_DISTANCE_EXPONENT = 2.0;
    public static final double RAMP_DOWN_DISTANCE_RAISED = Math.pow(RAMP_DOWN_DISTANCE, RAMP_DOWN_DISTANCE_EXPONENT);

    // Turn constants
    public static final double TURN_POWER = 0.3;
    public static final double MINIMUM_TURN_POWER = 0.2; // floor for power to the motors in a turn
    // Next line: If a turn is this close to its target, consider it done.
    public static final double TURN_THRESHOLD_DEGREES = 1.5; // As tight as we can make it with the gyro
    public static final double RAMP_DOWN_DEGREES = 15.0;
    public static final double RAMP_DOWN_DEGREES_EXPONENT = 2.0;
    public static final double RAMP_DOWN_DEGREES_RAISED = Math.pow(RAMP_DOWN_DISTANCE, RAMP_DOWN_DEGREES_EXPONENT);

    // PID constants
    public static final double P_DRIVE_COEFF = 0.03; // 0.05; // Larger is more responsive, but also less stable
    public static final double I_DRIVE_COEFF = P_DRIVE_COEFF / 10;
    public static final double P_TURN_COEFF = 0.05;  // Larger is more responsive, but also less stable

   // Stall constants
    public static final int MOTOR_MINIMUM_ENCODER_MOVEMENT = 5;
    public static final int MOTOR_MAXIMUM_STALL_COUNT = 5;

    public static final double HEADING_MINIMUM_MOVEMENT = 2.0;
    public static final int HEADING_MAXIMUM_STALL_COUNT = 5;

    public static final float FIELD_CENTER_TO_WALL = 1798.0f; // mm - as measured 1/16/2019 from the center of the field to the inner surface of the wall
    public static final float mmPerInch = 25.4f;
    public static final float mmFTCFieldWidth = FIELD_CENTER_TO_WALL; // Vuforia sample = (12 * 6) * mmPerInch = 1828.8 mm = the width of the FTC field (from the center point to the outer panels)
    public static final float mmTargetHeight = (6) * mmPerInch; // the height of the center of the target image above the floor

    public static final double GYRO_HEADING_NPOS = -361.0;

    public enum Alliance {
        BLUE, RED, UNKNOWN
    }

    // For each autonomous OpMode registered on the Robot Controller phone there
    // must a value in the enum OpMode.
    public enum OpMode {
        BLUE_OUTSIDE, BLUE_INSIDE, RED_OUTSIDE, RED_INSIDE, TEST
    }

    public enum TargetZone {
        TARGET_ZONE_A, TARGET_ZONE_B, TARGET_ZONE_C, TARGET_ZONE_NPOS
    }
}
