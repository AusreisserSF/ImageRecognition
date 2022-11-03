package org.firstinspires.ftc.teamcode.auto.vision;

//!! IntelliJ only

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.intellij.TimeStamp;
import org.firstinspires.ftc.ftcdevcommon.intellij.WorkingDirectory;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsPowerPlay;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.IOException;
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

//**TODO The bulk of ConeStackRecognition can be applied to color object
// recognition - gold cube and junction. Extract common code into
// RealSenseUtils. But be careful - this class takes ConeStackParameters,
// which includes parameters for red grayscale, red hsv and blue grayscale,
// blue hsv. May be better to generalize to either a single gray path and
// a single color path.
public class ConeStackRecognition {

    private static final String TAG = ConeStackRecognition.class.getSimpleName();

    private final String workingDirectory;
    private final ImageUtils imageUtils;
    private String outputFilenamePreamble;
    private Mat imageROI;
    private RobotConstants.Alliance alliance;
    private short[] depthArray;

    public ConeStackRecognition() {
        workingDirectory = WorkingDirectory.getWorkingDirectory() + RobotConstants.imageDir;
        imageUtils = new ImageUtils();
    }

    // Returns the result of image analysis.
    // The targets are:
    // A stack of 1 to 5 red cones
    // A stack of 1 to 5 blue cones
    public DepthReturn recognizeConeStack(ImageProvider pImageProvider,
                                          VisionParameters.ImageParameters pImageParameters,
                                          ConeStackParameters pConeStackParameters,
                                          RobotConstants.Alliance pAlliance,
                                          RobotConstantsPowerPlay.ConeStackRecognitionPath pConeStackRecognitionPath) throws InterruptedException, IOException {

        RobotLogCommon.d(TAG, "In ConeStackRecognition.recognizeConeStack");

        alliance = pAlliance;

        // LocalDateTime requires Android minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        Pair<Mat, LocalDateTime> coneStackImage = pImageProvider.getImage();
        if (coneStackImage == null)
            return new DepthReturn(RobotConstants.RecognitionResults.RECOGNITION_INTERNAL_ERROR); // don't crash

        // The image may be RGB (from a camera) or BGR (OpenCV imread from a file).
        // OpenCV wants BGR; the possible conversion is taken care of in imageUtils.preProcessImage.
        Mat imgOriginal = coneStackImage.first.clone();
        String fileDate = TimeStamp.getLocalDateTimeStamp(coneStackImage.second);
        outputFilenamePreamble = imageUtils.createOutputFilePreamble(pImageParameters.image_source, workingDirectory, RobotConstants.imageFilePrefix, fileDate);
        Mat imageROIOriginal = imageUtils.preProcessImage(pImageProvider, imgOriginal, outputFilenamePreamble, pImageParameters);

        // Subject the ROI to depth filtering on all paths.
        depthArray = RealSenseUtils.getDepthArrayFromFile(pImageParameters);
        imageROI = RealSenseUtils.removeBackground(imageROIOriginal, pImageParameters, depthArray,
                pConeStackParameters.depthParameters.minDepth,
                pConeStackParameters.depthParameters.maxDepth);

        Imgcodecs.imwrite(outputFilenamePreamble + "_ROI_RANGE.png", imageROI);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_ROI_RANGE.png");

        RobotLogCommon.d(TAG, "Recognition path " + pConeStackRecognitionPath);
        switch (pConeStackRecognitionPath) {
            case GRAYSCALE -> {
                return grayRecognitionPath(pImageParameters, pConeStackParameters);
            }
            case COLOR -> {
                return colorRecognitionPath(pImageParameters, pConeStackParameters);
            }
            default -> throw new AutonomousRobotException(TAG, "Unrecognized recogntion path");
        }
    }

    private DepthReturn grayRecognitionPath(VisionParameters.ImageParameters pImageParameters, ConeStackParameters pConeStackParameters) {
        // Remove distractions before we convert to grayscale: depending on the
        // current alliance set the red or blue channel pixels to black.
        ArrayList<Mat> channels = new ArrayList<>(3);
        Core.split(imageROI, channels); // red or blue channel. B = 0, G = 1, R = 2

        // For both the red cone and the blue cone use the red channel.
        // Then we'll threshold them differently because the color red
        // will be almost white in the grayscale image while the color
        // blue will be almost black.
        // Write out the red channel as grayscale.
        Imgcodecs.imwrite(outputFilenamePreamble + "_RED_CHANNEL.png", channels.get(2));
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_RED_CHANNEL.png");
        int grayMedianTarget = 0;
        int grayThresholdLow = 0;

        if (alliance == RobotConstants.Alliance.RED) {
            grayMedianTarget = pConeStackParameters.redGrayscaleParameters.median_target;
            grayThresholdLow = pConeStackParameters.redGrayscaleParameters.threshold_low;
        } else if (alliance == RobotConstants.Alliance.BLUE) {
            grayMedianTarget = pConeStackParameters.blueGrayscaleParameters.median_target;
            // The threshold value will be negative, which indicates that we should use
            // THRESH_BINARY_INV.
            grayThresholdLow = pConeStackParameters.blueGrayscaleParameters.threshold_low;
        }

        Mat thresholded = imageUtils.performThresholdOnGray(channels.get(2), outputFilenamePreamble, grayMedianTarget, grayThresholdLow);

        return RealSenseUtils.getAngleAndDepth(imageROI, thresholded, depthArray,
                outputFilenamePreamble, pImageParameters, pConeStackParameters);
    }

    //**TODO or use ImageUtils List<MatOfPoint> applyInRangeAndFindContours
    // and make an overload for RealSenseUtils.getAngleAndDepth
    private DepthReturn colorRecognitionPath(VisionParameters.ImageParameters pImageParameters, ConeStackParameters pConeStackParameters) {

        Mat thresholded;
        if (alliance == RobotConstants.Alliance.RED)
            thresholded = imageUtils.applyInRange(imageROI, outputFilenamePreamble, pConeStackParameters.redHSVParameters);
        else if (alliance == RobotConstants.Alliance.BLUE)
            thresholded = imageUtils.applyInRange(imageROI, outputFilenamePreamble, pConeStackParameters.blueHSVParameters);
        else
            return new DepthReturn(RobotConstants.RecognitionResults.RECOGNITION_INTERNAL_ERROR); // don't crash

        // Clean up the thresholded image via morphological opening.
        Mat morphed = new Mat();
        Imgproc.erode(thresholded, morphed, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));
        Imgproc.dilate(morphed, morphed, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));
        return RealSenseUtils.getAngleAndDepth(imageROI, thresholded, depthArray,
                outputFilenamePreamble, pImageParameters, pConeStackParameters);
    }

}