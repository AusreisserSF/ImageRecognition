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

import java.time.LocalDateTime;
import java.util.ArrayList;

public class SignalSleeveRecognition {

    private static final String TAG = SignalSleeveRecognition.class.getSimpleName();

    private final String workingDirectory;
    private final ImageUtils imageUtils;
    private String outputFilenamePreamble;
    private Mat imageROI;
    private RobotConstants.Alliance alliance;

    public SignalSleeveRecognition() {
        workingDirectory = WorkingDirectory.getWorkingDirectory() + RobotConstants.imageDir;
        imageUtils = new ImageUtils();
    }

    // Returns the result of image analysis.
    // The targets are:
    // Location 1: all black
    // Location 2: half-white, half black
    // Location 3: all white
    public SignalSleeveReturn recognizeSignalSleeve(ImageProvider pImageProvider,
                                                    VisionParameters.ImageParameters pImageParameters,
                                                    SignalSleeveParameters pSignalSleeveParameters,
                                                    RobotConstants.Alliance pAlliance,
                                                    RobotConstantsPowerPlay.SignalSleeveRecognitionPath pSignalSleeveRecognitionPath) throws InterruptedException {

        RobotLogCommon.d(TAG, "In SignalSleeveRecognition.recognizeSignalSleeve");

        alliance = pAlliance;

        // LocalDateTime requires Android minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        Pair<Mat, LocalDateTime> signalSleeveImage = pImageProvider.getImage();
        if (signalSleeveImage == null)
            return new SignalSleeveReturn(RobotConstants.OpenCVResults.OCV_ERROR); // don't crash

        // The image may be RGB (from a camera) or BGR (OpenCV imread from a file).
        // OpenCV wants BGR; the possible conversion is taken care of in imageUtils.preProcessImage.
        Mat imgOriginal = signalSleeveImage.first.clone();

        String fileDate = TimeStamp.getLocalDateTimeStamp(signalSleeveImage.second);
        outputFilenamePreamble = imageUtils.createOutputFilePreamble(pImageParameters.image_source, workingDirectory, RobotConstants.imageFilePrefix, fileDate);
        imageROI = imageUtils.preProcessImage(pImageProvider, imgOriginal, outputFilenamePreamble, pImageParameters);

        RobotLogCommon.d(TAG, "Recognition path " + pSignalSleeveRecognitionPath);
        SignalSleeveReturn retVal;
        switch (pSignalSleeveRecognitionPath) {
            case REFLECTIVE_TAPE -> retVal = reflectiveTapeRecognitionPath(pSignalSleeveParameters);
            case COLOR_SLEEVE -> retVal = colorSleeveRecognitionPath(pSignalSleeveParameters.colorSleeveParameters);
            default -> throw new AutonomousRobotException(TAG, "Unsupported recognition path " + pSignalSleeveRecognitionPath);
        }

        return retVal;
    }

    private SignalSleeveReturn reflectiveTapeRecognitionPath(SignalSleeveParameters pReflectiveTapeParameters) {
        ArrayList<Mat> channels = new ArrayList<>(3);
        Core.split(imageROI, channels);

        // For both the red cone and the blue cone use the red channel.
        // Then we'll threshold them differently because the color red
        // will be almost white in the grayscale image while the color
        // blue will be almost black.
        // Write out the red channel as grayscale.
        Imgcodecs.imwrite(outputFilenamePreamble + "_RED_CHANNEL.png", channels.get(2));
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_RED_CHANNEL.png");

        SignalSleeveParameters.GrayscaleParameters grayscaleParameters;
        Mat thresholded;

        if (alliance == RobotConstants.Alliance.RED) {
            grayscaleParameters = pReflectiveTapeParameters.redGrayscaleParameters;

            // Write out the red channel as grayscale.
            Imgcodecs.imwrite(outputFilenamePreamble + "_RED_CHANNEL.png", channels.get(2));
            RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_RED_CHANNEL.png");
       }
        else if (alliance == RobotConstants.Alliance.BLUE) {
            grayscaleParameters = pReflectiveTapeParameters.blueGrayscaleParameters;
       }
        else
            return new SignalSleeveReturn(RobotConstants.OpenCVResults.OCV_ERROR);

        // Always use the red channel - better contrast!
        thresholded = imageUtils.performThresholdOnGray(channels.get(2), outputFilenamePreamble, grayscaleParameters.grayParameters.median_target, grayscaleParameters.grayParameters.threshold_low);

        return getLocation(thresholded,
                grayscaleParameters.minWhitePixelsLocation2,
                grayscaleParameters.minWhitePixelsLocation3);
    }

    private SignalSleeveReturn colorSleeveRecognitionPath(SignalSleeveParameters.ColorSleeveParameters pColorSleeveParameters) {
        Mat thresholded = imageUtils.applyInRange(imageROI, outputFilenamePreamble, pColorSleeveParameters.hsvParameters);

        // Clean up the thresholded image via morphological opening.
        Mat morphed = new Mat();
        Imgproc.erode(thresholded, morphed, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));
        Imgproc.dilate(morphed, morphed, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));

        return getLocation(morphed,
                pColorSleeveParameters.minWhitePixelsLocation2,
                pColorSleeveParameters.minWhitePixelsLocation3);
    }

    //## 10/22/2022 Failed experiment. The chroma green tape is not distinct enough
    // from the gray tiles. However, the technique of splitting the channels should
    // prove useful in recognition of the red and blue cone stacks.
    /*
    private SignalSleeveReturn splitGreenRecognitionPath(SignalSleeveParameters.GrayscaleParameters pSplitGreenParameters) {
        // Remove distractions before we convert to grayscale: set the red and blue
        // channels to black and the green channel to white.
        ArrayList<Mat> channels = new ArrayList<>(3);

        Core.split(imageROI, channels);

        // Write out the green channel only as grayscale.
        Imgcodecs.imwrite(outputFilenamePreamble + "_GREEN_GRAY.png", channels.get(1));

        // B = 0, G = 1, R = 2.
        Mat blackChannel = Mat.zeros(channels.get(0).size(), CvType.CV_8UC1);
        Mat whiteChannel = Mat.ones(channels.get(0).size(), CvType.CV_8UC1);
        // Replace the blue channel with the black channel.
        //channels.set(0, blackChannel);
        // Replace the green channel with the white channel.
        //channels.set(1, whiteChannel);
        // Replace the red channel with the black channel.
        channels.set(2, blackChannel);

        // Put the BGR image back together.
        Core.merge(channels, imageROI);

        // Here's how to create an image with only the green channel.
        //Mat green = new Mat(480, 640, CvType.CV_8UC3, new Scalar(0,255,0));

        Imgcodecs.imwrite(outputFilenamePreamble + "_GREEN.png", imageROI);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_GREEN.png");

        // We're on the grayscale path.
        Mat grayROI = new Mat();
        Imgproc.cvtColor(imageROI, grayROI, Imgproc.COLOR_BGR2GRAY);

        Imgcodecs.imwrite(outputFilenamePreamble + "_GRAY.png", grayROI);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_GRAY.png");

        Mat adjustedGray = imageUtils.adjustGrayscaleBrightness(grayROI, pSplitGreenParameters.grayParameters.median_target);
        Imgcodecs.imwrite(outputFilenamePreamble + "_ADJ.png", adjustedGray);
        RobotLogCommon.d(TAG, "Writing adjusted grayscale image " + outputFilenamePreamble + "_ADJ.png");

        int grayThresholdLow = pSplitGreenParameters.grayParameters.threshold_low;
        RobotLogCommon.d(TAG, "Threshold value: low " + grayThresholdLow);

        //!! Wrong - Use ImageUtils.performThreshold()

        // Threshold the image: set pixels over the threshold value to white.
        Mat thresholded = new Mat(); // output binary image
        Imgproc.threshold(adjustedGray, thresholded,
                grayThresholdLow,    // threshold value
                255,   // white
                Imgproc.THRESH_BINARY); // thresholding type

        Imgcodecs.imwrite(outputFilenamePreamble + "_ADJ_THR.png", thresholded);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_ADJ_THR.png");

        return getLocation(thresholded,
                pSplitGreenParameters.minWhitePixelsLocation2,
                pSplitGreenParameters.minWhitePixelsLocation3);
    }
     */

    private SignalSleeveReturn getLocation(Mat pThresholded, int pMinWhitePixelsLocation2, int pMinWhitePixelsLocation3) {
        // Our target,unless it's location 1, which is black, will now appear
        // white in the thresholded image.
        int nonZeroPixelCount = Core.countNonZero(pThresholded);
        RobotLogCommon.d(TAG, "Number of non-zero pixels " + nonZeroPixelCount);

        // Check the minimum non-zero pixel count for the sleeve with the greatest
        // number of white pixels, location 3.
        RobotLogCommon.d(TAG, "Minimum non-zero-pixel count for location 3 " + pMinWhitePixelsLocation3);
        if (nonZeroPixelCount > pMinWhitePixelsLocation3) {
            RobotLogCommon.d(TAG, "The signal sleeve indicates location 3.");
            return new SignalSleeveReturn(RobotConstants.OpenCVResults.RECOGNITION_SUCCESSFUL, RobotConstantsPowerPlay.SignalSleeveLocation.LOCATION_3);
        }

        // Try location 2.
        RobotLogCommon.d(TAG, "Minimum non-zero-pixel count for location 2 " + pMinWhitePixelsLocation2);
        if (nonZeroPixelCount > pMinWhitePixelsLocation2) {
            RobotLogCommon.d(TAG, "The signal sleeve indicates location 2.");
            return new SignalSleeveReturn(RobotConstants.OpenCVResults.RECOGNITION_SUCCESSFUL, RobotConstantsPowerPlay.SignalSleeveLocation.LOCATION_2);
        }

        // Default: must be location 1.
        return new SignalSleeveReturn(RobotConstants.OpenCVResults.RECOGNITION_SUCCESSFUL, RobotConstantsPowerPlay.SignalSleeveLocation.LOCATION_1);

    }
}