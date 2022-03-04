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
        public final double calibrationObjectDistance;
        public final double calibrationObjectWidth;
        public final double focalLength;
        public final double cameraToRobotCenter; // inches

        public DistanceParameters(double pCalibrationDistance, double pCalibrationWidth,
                                  double pFocalLength, double pCameraToRobotCenter) {
            calibrationObjectDistance = pCalibrationDistance;
            calibrationObjectWidth = pCalibrationWidth;
            focalLength = pFocalLength;
            cameraToRobotCenter = pCameraToRobotCenter;
        }
    }

}