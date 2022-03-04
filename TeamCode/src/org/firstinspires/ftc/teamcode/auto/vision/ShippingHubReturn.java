package org.firstinspires.ftc.teamcode.auto.vision;

// Holds the results of image recognition.
public class ShippingHubReturn {

    public final boolean fatalComputerVisionError;
    public final double angleToShippingHub;
    public final double distanceToShippingHub;

    public ShippingHubReturn(boolean pFatalComputerVisionError, double pAngle, double pDistance) {
        fatalComputerVisionError = pFatalComputerVisionError;
        angleToShippingHub = pAngle;
        distanceToShippingHub = pDistance;
    }

}