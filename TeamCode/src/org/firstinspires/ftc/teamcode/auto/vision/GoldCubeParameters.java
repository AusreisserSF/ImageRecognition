package org.firstinspires.ftc.teamcode.auto.vision;

// Input parameters to barcode recognition.
public class GoldCubeParameters {

    public final VisionParameters.HSVParameters hsvParameters;
    public final float depthCameraScale;
    public final float depthCameraDistanceFilter;

    public GoldCubeParameters(VisionParameters.HSVParameters pHSVParameters,
                              float pDepthCameraScale, float pDepthCameraDistanceFilter) {
        hsvParameters = pHSVParameters;
        depthCameraScale = pDepthCameraScale;
        depthCameraDistanceFilter = pDepthCameraDistanceFilter;
    }

}