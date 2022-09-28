package org.firstinspires.ftc.teamcode.auto.vision;

// Input parameters to recognition using an Intel RealSense camera.
public class RealSenseParameters {

    public final VisionParameters.HSVParameters hsvParameters;
    public final float depthCameraFOV;
    //**TODO See OneNote - depth scale for D4xx cameras is fixed at .001
    public final float depthCameraScale;
    public final float depthCameraDistanceFilter;
    public final float cameraToRobotCenterMeters;

    public RealSenseParameters(VisionParameters.HSVParameters pHSVParameters,
                               float pColorCameraFOV, float pDepthCameraScale, float pDepthCameraDistanceFilter,
                               float pCameraToRobotCenterMeters) {
        hsvParameters = pHSVParameters;
        depthCameraFOV = pColorCameraFOV;
        depthCameraScale = pDepthCameraScale;
        depthCameraDistanceFilter = pDepthCameraDistanceFilter;
        cameraToRobotCenterMeters = pCameraToRobotCenterMeters;
    }

}