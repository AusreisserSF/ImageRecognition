package org.firstinspires.ftc.teamcode.auto.vision;

// Input parameters for cone stack recognition.
// Split color channels -> grayscale
public class ConeStackParameters {

    public final VisionParameters.GrayParameters redGrayscaleParameters;

    public ConeStackParameters(VisionParameters.GrayParameters pRedGrayscaleParameters) {
        redGrayscaleParameters = pRedGrayscaleParameters;
    }

}