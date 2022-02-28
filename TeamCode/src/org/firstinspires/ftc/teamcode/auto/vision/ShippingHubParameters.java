package org.firstinspires.ftc.teamcode.auto.vision;

// Input parameters to Shipping Hub recognition.
public class ShippingHubParameters {

    public final VisionParameters.HSVParameters blueAllianceHSVParameters;
    public final VisionParameters.HSVParameters redAllianceHSVParameters;
    public final DistanceParameters distanceParameters;

    public ShippingHubParameters(VisionParameters.HSVParameters pBlueAllianceHSVParameters,
                                 VisionParameters.HSVParameters pRedAllianceHSVParameters,
                                 DistanceParameters pDistanceParameters) {
        blueAllianceHSVParameters = pBlueAllianceHSVParameters;
        redAllianceHSVParameters = pRedAllianceHSVParameters;
        distanceParameters = pDistanceParameters;
    }

    public static class DistanceParameters {
        public final double known_distance;
        public final double known_width;
        public final double focal_length;

        public DistanceParameters(double pKnownDistance, double pKnownWidth, double pFocalWidth) {
            known_distance = pKnownDistance;
            known_width = pKnownWidth;
            focal_length = pFocalWidth;
        }
    }

}