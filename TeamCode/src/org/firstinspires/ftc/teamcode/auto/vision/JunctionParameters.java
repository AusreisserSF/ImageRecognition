package org.firstinspires.ftc.teamcode.auto.vision;

// Input parameters for junction recognition.
public class JunctionParameters {

    public final VisionParameters.GrayParameters junctionCapGrayscaleParameters;
    public final VisionParameters.GrayParameters junctionPoleGrayscaleParameters;
    public final VisionParameters.HSVParameters junctionPoleHsvParameters;
    public final DepthParameters depthParameters;

    public JunctionParameters(VisionParameters.GrayParameters pJunctionCapGrayscaleParameters,
                              VisionParameters.GrayParameters pJunctionPoleGrayscaleParameters,
                              VisionParameters.HSVParameters pJunctionPoleHSVParameters,
                              DepthParameters pDepthParameters) {
        junctionCapGrayscaleParameters = pJunctionCapGrayscaleParameters;
        junctionPoleGrayscaleParameters = pJunctionPoleGrayscaleParameters;
        junctionPoleHsvParameters = pJunctionPoleHSVParameters;
        depthParameters = pDepthParameters;
    }

}