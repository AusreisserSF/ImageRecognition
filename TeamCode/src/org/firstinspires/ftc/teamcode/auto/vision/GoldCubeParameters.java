package org.firstinspires.ftc.teamcode.auto.vision;

// Input parameters to barcode recognition.
public class GoldCubeParameters {

    public final VisionParameters.HSVParameters hsvParameters;
    public final float colorCameraFOV;
    public final float depthCameraScale;
    public final float depthCameraDistanceFilter;
    public final float cameraToRobotCenterMeters;

    public GoldCubeParameters(VisionParameters.HSVParameters pHSVParameters,
                              float pColorCameraFOV, float pDepthCameraScale, float pDepthCameraDistanceFilter,
                              float pCameraToRobotCenterMeters) {
        hsvParameters = pHSVParameters;
        colorCameraFOV = pColorCameraFOV;
        depthCameraScale = pDepthCameraScale;
        depthCameraDistanceFilter = pDepthCameraDistanceFilter;
        cameraToRobotCenterMeters = pCameraToRobotCenterMeters;
    }

}