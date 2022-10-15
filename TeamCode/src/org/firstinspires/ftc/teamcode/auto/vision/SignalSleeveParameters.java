package org.firstinspires.ftc.teamcode.auto.vision;

// Input parameters for signal sleeve recognition.
// Reflective tape -> grayscale
// Color sleeve -> HSV
public class SignalSleeveParameters {

    public final ReflectiveTapeParameters reflectiveTapeParameters;
    public final ColorSleeveParameters colorSleeveParameters;

    public SignalSleeveParameters(ReflectiveTapeParameters pReflectiveTapeParameters,
                                  ColorSleeveParameters pColorSleeveParameters) {
        reflectiveTapeParameters = pReflectiveTapeParameters;
        colorSleeveParameters = pColorSleeveParameters;
    }

    public static class ReflectiveTapeParameters {
        public final VisionParameters.GrayParameters grayParameters;
        public final int minWhitePixelsLocation2;
        public final int minWhitePixelsLocation3;

        public ReflectiveTapeParameters(VisionParameters.GrayParameters pGrayParameters,
                                        int pMinWhitePixelsLocation2,
                                        int pMinWhitePixelsLocation3) {
            grayParameters = pGrayParameters;
            minWhitePixelsLocation2 = pMinWhitePixelsLocation2;
            minWhitePixelsLocation3 = pMinWhitePixelsLocation3;
        }
    }

    public static class ColorSleeveParameters {
        public final VisionParameters.HSVParameters hsvParameters;
        public final int minWhitePixelsLocation2;
        public final int minWhitePixelsLocation3;

        public ColorSleeveParameters(VisionParameters.HSVParameters pHSVParameters,
                                     int pMinWhitePixelsLocation2,
                                     int pMinWhitePixelsLocation3) {
            hsvParameters = pHSVParameters;
            minWhitePixelsLocation2 = pMinWhitePixelsLocation2;
            minWhitePixelsLocation3 = pMinWhitePixelsLocation3;
        }
    }

}