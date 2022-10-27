package org.firstinspires.ftc.teamcode.auto.vision;

import org.opencv.core.Rect;

// Input parameters for cone stack recognition.
// Split color channels -> grayscale
public class ConeStackParameters {

    public final VisionParameters.GrayParameters redGrayscaleParameters;
    public final VisionParameters.GrayParameters blueGrayscaleParameters;
    public final DepthParameters depthParameters;

    public ConeStackParameters(VisionParameters.GrayParameters pRedGrayscaleParameters,
                               VisionParameters.GrayParameters pBlueGrayscaleParameters,
                               DepthParameters pDepthParameters) {
        redGrayscaleParameters = pRedGrayscaleParameters;
        blueGrayscaleParameters = pBlueGrayscaleParameters;
        depthParameters = pDepthParameters;
    }

    public static class DepthParameters {
        public final Rect depthWindowOffsets;
        public final float minDepth;
        public final float maxDepth;

        DepthParameters(int pDepthWindowOffsetX, int pDepthWindowOffsetY,
                        int pDepthWindowWidth, int pDepthWindowHeight,
                        float pMinDepth, float pMaxDepth) {
            depthWindowOffsets = new Rect(pDepthWindowOffsetX, pDepthWindowOffsetY,
                    pDepthWindowWidth, pDepthWindowHeight);
            minDepth = pMinDepth;
            maxDepth = pMaxDepth;
        }
    }

}