package org.firstinspires.ftc.teamcode.auto.vision;

// Holds the results of image recognition.
public class ShippingHubAngleReturn {

    public final boolean fatalComputerVisionError;
    public final double angleToShippingHub;

    public ShippingHubAngleReturn(boolean pFatalComputerVisionError, double pAngle) {
        fatalComputerVisionError = pFatalComputerVisionError;
        angleToShippingHub = pAngle;
    }

}