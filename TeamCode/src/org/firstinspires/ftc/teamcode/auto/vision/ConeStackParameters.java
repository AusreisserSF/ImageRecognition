package org.firstinspires.ftc.teamcode.auto.vision;

// Input parameters for cone stack recognition.
// Split color channels -> grayscale
public class ConeStackParameters {

    public final VisionParameters.GrayParameters redGrayscaleParameters;
    public final VisionParameters.HSVParameters redHSVParameters;
    public final VisionParameters.GrayParameters blueGrayscaleParameters;
    public final VisionParameters.HSVParameters blueHSVParameters;

    public final DepthParameters depthParameters;

    public ConeStackParameters(VisionParameters.GrayParameters pRedGrayscaleParameters,
                               VisionParameters.HSVParameters pRedHSVParameters,
                               VisionParameters.GrayParameters pBlueGrayscaleParameters,
                               VisionParameters.HSVParameters pBlueHSVParameters,
                               DepthParameters pDepthParameters) {
        redGrayscaleParameters = pRedGrayscaleParameters;
        redHSVParameters = pRedHSVParameters;
        blueGrayscaleParameters = pBlueGrayscaleParameters;
        blueHSVParameters = pBlueHSVParameters;
        depthParameters = pDepthParameters;
    }

}