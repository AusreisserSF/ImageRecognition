package org.firstinspires.ftc.teamcode.auto.vision;

//!! IntelliJ only

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.intellij.TimeStamp;
import org.firstinspires.ftc.ftcdevcommon.intellij.WorkingDirectory;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsPowerPlay;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
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
    private int minWhitePixelsLocation2;
    private int minWhitePixelsLocation3;

    public SignalSleeveRecognition() {
        workingDirectory = WorkingDirectory.getWorkingDirectory() + RobotConstants.imageDir;
        imageUtils = new ImageUtils();
    }

    // Returns the result of image analysis.
    // The targets are:
    // Location 1: all black
    // Location 2: half-white, half blakc
    // Location 3: all white
    public SignalSleeveReturn recognizeSignalSleeve(ImageProvider pImageProvider,
                                                    VisionParameters.ImageParameters pImageParameters,
                                                    SignalSleeveParameters pSignalSleeveParameters,
                                                    RobotConstants.Alliance pAlliance,
    RobotConstantsPowerPlay.RecognitionPath pRecognitionPath) throws InterruptedException {

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

        // Set the minimum pixel counts for recognition.
        minWhitePixelsLocation2 = pSignalSleeveParameters.minWhitePixelsLocation2;
        minWhitePixelsLocation3 = pSignalSleeveParameters.minWhitePixelsLocation3;

        RobotLogCommon.d(TAG, "Recognition path " + pRecognitionPath);
        SignalSleeveReturn retVal;
        switch (pRecognitionPath) {
            case REFLECTIVE_TAPE: {
                retVal = reflectiveTapeRecognitionPath(pSignalSleeveParameters.grayParameters);
                break;
            }
            case HSV: {
                retVal = hsvRecognitionPath(pSignalSleeveParameters.hsvParameters);
                break;
            }
            default:
                throw new AutonomousRobotException(TAG, "Unsupported recognition path " + pRecognitionPath);
        }

        return retVal;
    }

        private SignalSleeveReturn reflectiveTapeRecognitionPath(VisionParameters.GrayParameters pGrayParameters) {
        // Remove distractions before we convert to grayscale: depending on the
        // current alliance set the red or blue channel pixels to black.
        ArrayList<Mat> channels = new ArrayList<>(3);
        Core.split(imageROI, channels);

        // Initialize a new Mat to black (all zeros) to take the place of the
        // red or blue channel.
        Mat blackChannel = Mat.zeros(imageROI.rows(), imageROI.cols(), CvType.CV_8U);
        if (alliance == RobotConstants.Alliance.RED) {
            // Remove the red channel and substitute the black channel.
            channels.remove(2); // B = 0, G = 1, R = 2
            channels.add(blackChannel);
            Core.merge(channels, imageROI); // Recreate the BGR image.
        } else if (alliance == RobotConstants.Alliance.BLUE) {
            // Remove the blue channel and substitute the black channel.
            channels.remove(0); // B = 0, G = 1, R = 2
            channels.add(0, blackChannel);
            Core.merge(channels, imageROI); // Recreate the BGR image.
        }

        // We're on the grayscale path.
        Mat grayROI = new Mat();
        Imgproc.cvtColor(imageROI, grayROI, Imgproc.COLOR_BGR2GRAY);

        Imgcodecs.imwrite(outputFilenamePreamble + "_REF_GRAY.png", grayROI);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_GRAY.png");

        Mat adjustedGray = imageUtils.adjustGrayscaleBrightness(grayROI, pGrayParameters.target);
        Imgcodecs.imwrite(outputFilenamePreamble + "_REF_ADJ.png", adjustedGray);
        RobotLogCommon.d(TAG, "Writing adjusted grayscale image " + outputFilenamePreamble + "_REF_ADJ.png");

        int grayThresholdLow = pGrayParameters.low_threshold;
        RobotLogCommon.d(TAG, "Threshold value: low " + grayThresholdLow);

        // Threshold the image: set pixels over the threshold value to white.
        Mat thresholded = new Mat(); // output binary image
        Imgproc.threshold(adjustedGray, thresholded,
                grayThresholdLow,    // threshold value
                255,   // white
                Imgproc.THRESH_BINARY); // thresholding type

        return getLocation(thresholded);
   }

    private SignalSleeveReturn hsvRecognitionPath(VisionParameters.HSVParameters pHSVParameters) {
        Mat thresholded = imageUtils.applyInRange(imageROI, outputFilenamePreamble, pHSVParameters);

        // Clean up the thresholded image via morphological opening.
        Mat morphed = new Mat();
        Imgproc.erode(thresholded, morphed, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));
        Imgproc.dilate(morphed, morphed, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));

        return getLocation(thresholded);
    }

        private SignalSleeveReturn getLocation(Mat pThresholded) {
        // Our target,unless it's location 1, which is black, will now appear
        // white in the thresholded image.
        Imgcodecs.imwrite(outputFilenamePreamble + "_REF_ADJ_THR.png", pThresholded);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_REF_ADJ_THR.png");

        int nonZeroPixelCount = Core.countNonZero(pThresholded);
        RobotLogCommon.d(TAG, "Number of non-zero pixels " + nonZeroPixelCount);

        // Check the minimum non-zero pixel count for the sleeve with the greatest
        // number of white pixels, location 3.
        RobotLogCommon.d(TAG, "Minimum non-zero-pixel count for location 3 " + minWhitePixelsLocation3);
        if (nonZeroPixelCount > minWhitePixelsLocation3) {
            RobotLogCommon.d(TAG, "The signal sleeve indicates location 3.");
            return new SignalSleeveReturn(RobotConstants.OpenCVResults.RECOGNITION_SUCCESSFUL, RobotConstantsPowerPlay.SignalSleeveLocation.LOCATION_3);
        }

        // Try location 2.
        RobotLogCommon.d(TAG, "Minimum non-zero-pixel count for location 2 " + minWhitePixelsLocation2);
        if (nonZeroPixelCount > minWhitePixelsLocation2) {
            RobotLogCommon.d(TAG, "The signal sleeve indicates location 2.");
            return new SignalSleeveReturn(RobotConstants.OpenCVResults.RECOGNITION_SUCCESSFUL, RobotConstantsPowerPlay.SignalSleeveLocation.LOCATION_2);
        }

        // Default: must be location 1.
        return new SignalSleeveReturn(RobotConstants.OpenCVResults.RECOGNITION_SUCCESSFUL, RobotConstantsPowerPlay.SignalSleeveLocation.LOCATION_1);

    }
}