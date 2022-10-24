package org.firstinspires.ftc.teamcode.auto.vision;

// Input parameters for cone stack recognition.
// Split color channels -> grayscale
public class ConeStackParameters {

    public final VisionParameters.GrayParameters redGrayscaleParameters;
    public final VisionParameters.GrayParameters blueGrayscaleParameters;

    public ConeStackParameters(VisionParameters.GrayParameters pRedGrayscaleParameters,
                               VisionParameters.GrayParameters pBlueGrayscaleParameters) {
        redGrayscaleParameters = pRedGrayscaleParameters;
        blueGrayscaleParameters = pBlueGrayscaleParameters;
    }

}