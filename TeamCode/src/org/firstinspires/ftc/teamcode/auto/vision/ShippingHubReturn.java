package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsFreightFrenzy;

// Holds the results of image recognition.
public class ShippingHubReturn {

    public static final double SHIPPING_HUB_ANGLE_NPOS = -360.0;
    public static final double SHIPPING_HUB_DISTANCE_NPOS = -1.0;

    public final RobotConstants.OpenCVResults openCVResults;
    public final double angleToShippingHub;
    public final double distanceToShippingHub;

    public ShippingHubReturn(RobotConstants.OpenCVResults pOpenCVResults, double pAngle, double pDistance) {
        openCVResults = pOpenCVResults;
        angleToShippingHub = pAngle;
        distanceToShippingHub = pDistance;
    }

    // Constructor for OpenCV errors such as "no such file".
    public ShippingHubReturn(RobotConstants.OpenCVResults pOpenCVResults) {
        openCVResults = pOpenCVResults;
        angleToShippingHub = SHIPPING_HUB_ANGLE_NPOS;
        distanceToShippingHub = SHIPPING_HUB_DISTANCE_NPOS;
    }
}