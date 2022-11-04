package org.firstinspires.ftc.teamcode.auto.vision;

// Input parameters for junction recognition.
public class JunctionParameters {

    public final VisionParameters.GrayParameters grayscaleParameters;
    public final VisionParameters.HSVParameters hsvParameters;
    public final DepthParameters depthParameters;

    public JunctionParameters(VisionParameters.GrayParameters pGrayscaleParameters,
                              VisionParameters.HSVParameters pHSVParameters,
                              DepthParameters pDepthParameters) {
        grayscaleParameters = pGrayscaleParameters;
        hsvParameters = pHSVParameters;
        depthParameters = pDepthParameters;
    }

}