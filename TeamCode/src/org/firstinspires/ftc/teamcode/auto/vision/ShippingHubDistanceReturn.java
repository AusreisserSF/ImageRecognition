package org.firstinspires.ftc.teamcode.auto.vision;

// Holds the results of image recognition.
public class ShippingHubDistanceReturn {

    public final boolean fatalComputerVisionError;
    public final double distanceToShippingHub;

    public ShippingHubDistanceReturn(boolean pFatalComputerVisionError, double pDistance) {
        fatalComputerVisionError = pFatalComputerVisionError;
        distanceToShippingHub = pDistance;
    }

}