package org.firstinspires.ftc.teamcode.auto.vision;

import org.opencv.core.Rect;

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

    public static class DepthParameters {
        // These depth values are given as percentages of the size of the
        // bounding rectangle which encloses the contour of a cone.
        public final int depthWindowOffsetX;
        public final int depthWindowOffsetY;
        public final int depthWindowWidth;
        public final int depthWindowHeight;

        public final float minDepth;
        public final float maxDepth;

        //## Note: the parameter pDepthWindowOffsetX is determined as follows:
        // From the x-coordinate of the center of the bounding box subtract this
        // number, which is given as a percentage of the width of the bounding
        // box. Double the result to get the width of the depth window. -->
        public DepthParameters(int pDepthWindowOffsetX, int pDepthWindowOffsetY,
                               int pDepthWindowHeight,
                               float pMinDepth, float pMaxDepth) {
            depthWindowOffsetX = pDepthWindowOffsetX;
            depthWindowOffsetY = pDepthWindowOffsetY;
            depthWindowWidth = pDepthWindowOffsetX * 2;
            depthWindowHeight = pDepthWindowHeight;
            minDepth = pMinDepth;
            maxDepth = pMaxDepth;
        }
    }

}