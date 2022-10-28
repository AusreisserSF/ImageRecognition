package org.firstinspires.ftc.teamcode.auto.vision;

// Input parameters to recognition using an Intel RealSense camera.
public class RealSenseParameters {

    public final VisionParameters.HSVParameters hsvParameters;
    public final float depthCameraFOV; //**TODO RobotConstants
    public final float depthCameraScale; //**TODO RobotConstants
    public final float depthCameraDistanceFilter;
    public final float cameraToRobotCenterMeters; //**TODO RobotConstants

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