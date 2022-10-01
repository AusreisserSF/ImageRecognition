package org.firstinspires.ftc.teamcode.auto.vision;

// Input parameters to OpenCV grayscale signal sleeve recognition.
public class SignalSleeveParameters {

    public final VisionParameters.GrayParameters grayParameters;
    public final int minWhitePixelsLocation2;
    public final int minWhitePixelsLocation3;

    public SignalSleeveParameters(VisionParameters.GrayParameters pGrayParameters,
                                  int pMinWhitePixelsLocation2,
                                  int pMinWhitePixelsLocation3) {
        grayParameters = pGrayParameters;
        minWhitePixelsLocation2 = pMinWhitePixelsLocation2;
        minWhitePixelsLocation3 = pMinWhitePixelsLocation3;
    }

}