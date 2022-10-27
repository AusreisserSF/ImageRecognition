package org.firstinspires.ftc.teamcode.auto.vision;

// Input parameters to recognition using an Intel RealSense camera.
//**TODO Need (RealSense)D405Parameters (source = XML) for all of
// the fields below except hsv.
public class RealSenseParameters {

    public final VisionParameters.HSVParameters hsvParameters;
    public final float depthCameraFOV;
    public final float depthCameraScale;
    //**TODO Need distanceFilterMin and distanceFilterMax
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