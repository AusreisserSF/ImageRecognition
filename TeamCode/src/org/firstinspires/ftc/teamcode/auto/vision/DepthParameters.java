package org.firstinspires.ftc.teamcode.auto.vision;

public class DepthParameters {
    // These depth values are given as percentages of the size of the
    // bounding rectangle which encloses the contour of an object.
    public final double depthWindowOffsetX;
    public final double depthWindowOffsetY;
    public final double depthWindowWidth;
    public final double depthWindowHeight;

    public final float minDepth;
    public final float maxDepth;

    //## Note: the parameter pDepthWindowOffsetX is determined as follows:
    // From the x-coordinate of the center of the bounding box subtract this
    // number, which is given as a percentage of the width of the bounding
    // box. Double the result to get the width of the depth window. -->
    public DepthParameters(double pDepthWindowOffsetX, double pDepthWindowOffsetY,
                           double pDepthWindowHeight,
                           float pMinDepth, float pMaxDepth) {
        depthWindowOffsetX = pDepthWindowOffsetX;
        depthWindowOffsetY = pDepthWindowOffsetY;
        depthWindowWidth = pDepthWindowOffsetX * 2;
        depthWindowHeight = pDepthWindowHeight;
        minDepth = pMinDepth;
        maxDepth = pMaxDepth;
    }
}
