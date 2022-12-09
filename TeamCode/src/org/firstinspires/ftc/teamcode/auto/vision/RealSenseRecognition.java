package org.firstinspires.ftc.teamcode.auto.vision;

//!! IntelliJ only

import org.firstinspires.ftc.ftcdevcommon.intellij.RobotLogCommon;
import org.firstinspires.ftc.teamcode.common.RobotConstantsPowerPlay;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class RealSenseRecognition {

    private static final String TAG = RealSenseRecognition.class.getSimpleName();

    public RealSenseRecognition() {}

    // Analyzes a grayscale image using only the red channel.
    public RealSenseReturn redChannelPath(Mat pImageROI,
                                          D405Configuration pD405Configuration,
                                          RobotConstantsPowerPlay.D405CameraId pOrientation,
                                          short[] pDepthArray,
                                          String pOutputFilenamePreamble,
                                          VisionParameters.ImageParameters pImageParameters,
                                          VisionParameters.GrayParameters pGrayParameters,
                                          DepthParameters pDepthParameters) {

        // Remove distractions before we convert to grayscale: depending on the
        // current alliance set the red or blue channel pixels to black.
        ArrayList<Mat> channels = new ArrayList<>(3);
        Core.split(pImageROI, channels); // red or blue channel. B = 0, G = 1, R = 2

        // For both the red cone and the blue cone use the red channel.
        // Then we'll threshold them differently because the color red
        // will be almost white in the grayscale image while the color
        // blue will be almost black.
        // Write out the red channel as grayscale.
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_RED_CHANNEL.png", channels.get(2));
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_RED_CHANNEL.png");

        Mat thresholded = ImageUtils.performThresholdOnGray(channels.get(2), pOutputFilenamePreamble, pGrayParameters.median_target, pGrayParameters.threshold_low);

        return RealSenseUtils.getAngleAndDistance(pImageROI, thresholded,
                pD405Configuration, pOrientation, pDepthArray,
                pOutputFilenamePreamble, pImageParameters, pDepthParameters);
    }

    // Analyzes a color image.
    public RealSenseReturn colorPath(Mat pImageROI,
                                     D405Configuration pD405Configuration,
                                     RobotConstantsPowerPlay.D405CameraId pOrientation,
                                     short[] pDepthArray,
                                     String pOutputFilenamePreamble,
                                     VisionParameters.ImageParameters pImageParameters,
                                     VisionParameters.HSVParameters pHSVParameters,
                                     DepthParameters pDepthParameters) {

        Mat thresholded = ImageUtils.applyInRange(pImageROI, pOutputFilenamePreamble, pHSVParameters);

        // Clean up the thresholded image via morphological opening.
        Mat morphed = new Mat();
        Imgproc.erode(thresholded, morphed, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));
        Imgproc.dilate(morphed, morphed, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));

        return RealSenseUtils.getAngleAndDistance(pImageROI, thresholded,
                pD405Configuration, pOrientation, pDepthArray,
                pOutputFilenamePreamble, pImageParameters, pDepthParameters);
    }
}