package org.firstinspires.ftc.teamcode.auto.vision;

// Input parameters to OpenCV grayscale signal sleeve recognition.
public class SignalSleeveParameters {

    public final VisionParameters.GrayParameters grayParameters;
    public final VisionParameters.HSVParameters hsvParameters;
    public final int minWhitePixelsLocation2;
    public final int minWhitePixelsLocation3;

    public SignalSleeveParameters(VisionParameters.GrayParameters pGrayParameters,
                                  VisionParameters.HSVParameters pHSVParameters,
                                  int pMinWhitePixelsLocation2,
                                  int pMinWhitePixelsLocation3) {
        grayParameters = pGrayParameters;
        hsvParameters = pHSVParameters;
        minWhitePixelsLocation2 = pMinWhitePixelsLocation2;
        minWhitePixelsLocation3 = pMinWhitePixelsLocation3;
    }

}