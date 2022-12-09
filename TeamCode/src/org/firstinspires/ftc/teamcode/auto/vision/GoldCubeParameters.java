package org.firstinspires.ftc.teamcode.auto.vision;

// Input parameters to gold cube recognition.
public class GoldCubeParameters {

    public final VisionParameters.GrayParameters grayscaleParameters;
    public final VisionParameters.HSVParameters hsvParameters;
    public final DepthParameters depthParameters;

    public GoldCubeParameters(VisionParameters.GrayParameters pGrayscaleParameters,
                              VisionParameters.HSVParameters pHSVParameters,
                              DepthParameters pDepthParameters) {
        grayscaleParameters = pGrayscaleParameters;
        hsvParameters = pHSVParameters;
        depthParameters = pDepthParameters;
    }

}