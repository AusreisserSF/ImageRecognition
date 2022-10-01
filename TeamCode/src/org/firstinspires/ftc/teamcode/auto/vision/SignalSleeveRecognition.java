package org.firstinspires.ftc.teamcode.auto.vision;

//!! IntelliJ only

import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.intellij.TimeStamp;
import org.firstinspires.ftc.ftcdevcommon.intellij.WorkingDirectory;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsPowerPlay;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.time.LocalDateTime;
import java.util.Date;

public class SignalSleeveRecognition {

    private static final String TAG = SignalSleeveRecognition.class.getSimpleName();

    private final String workingDirectory;
    private final ImageUtils imageUtils;

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
                                                    RobotConstantsPowerPlay.RecognitionPath pRecognitionPath) throws InterruptedException {

        RobotLogCommon.d(TAG, "In SignalSleeveRecognition.recognizeSignalSleeve");

        // LocalDateTime requires Android minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        Pair<Mat, LocalDateTime> signalSleeveImage = pImageProvider.getImage();
        if (signalSleeveImage.first == null)
            return new SignalSleeveReturn(RobotConstants.OpenCVResults.OCV_ERROR); // don't crash

        // The image may be RGB (from a camera) or BGR (OpenCV imread from a file).
        Mat imgOriginal = signalSleeveImage.first.clone();

        String fileDate = TimeStamp.getLocalDateTimeStamp(signalSleeveImage.second);
        String outputFilenamePreamble = imageUtils.createOutputFilePreamble(pImageParameters.ocv_image, workingDirectory, RobotConstants.imageFilePrefix, fileDate);
        Mat imageROI = imageUtils.preProcessImage(pImageProvider, imgOriginal, outputFilenamePreamble, pImageParameters);

        // Set the minimum pixel counts for recognition.
        int minWhitePixelsLocation2 = pSignalSleeveParameters.minWhitePixelsLocation2;
        int minWhitePixelsLocation3 = pSignalSleeveParameters.minWhitePixelsLocation3;

        RobotLogCommon.d(TAG, "Recognition path " + pRecognitionPath);

        // We're on the grayscale path.
        Mat grayROI = new Mat();
        Imgproc.cvtColor(imageROI, grayROI, Imgproc.COLOR_BGR2GRAY);

        //Imgcodecs.imwrite(outputFilenamePreamble + "_REF_GRAY.png", grayROI);
        //RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_GRAY.png");

        Mat adjustedGray = imageUtils.adjustGrayscaleBrightness(grayROI, pSignalSleeveParameters.grayParameters.target);
        //Imgcodecs.imwrite(outputFilenamePreamble + "_REF_ADJ.png", adjustedGray);
        //RobotLogCommon.d(TAG, "Writing adjusted grayscale image " + outputFilenamePreamble + "_REF_ADJ.png");

        int grayThresholdLow = pSignalSleeveParameters.grayParameters.low_threshold;
        RobotLogCommon.d(TAG, "Threshold value: low " + grayThresholdLow);

        // Threshold the image: set pixels over the threshold value to white.
        Mat thresholded = new Mat(); // output binary image
        Imgproc.threshold(adjustedGray, thresholded,
                grayThresholdLow,    // threshold value
                255,   // white
                Imgproc.THRESH_BINARY); // thresholding type

        // Our target,unless it's location 1, which is black, will now appear
        // white in the thresholded image.
        Imgcodecs.imwrite(outputFilenamePreamble + "_REF_ADJ_THR.png", thresholded);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_REF_ADJ_THR.png");

        int nonZeroPixelCount = Core.countNonZero(thresholded);
        RobotLogCommon.d(TAG, "Number of non-zero pixels " + nonZeroPixelCount);

        // Check the minimum non-zero pixel count for the sleeve with the greatest
        // number of white pixels, location 3.
        RobotLogCommon.d(TAG, "Minimum non-zero-pixel count for location 3 " + minWhitePixelsLocation3);
        if (nonZeroPixelCount > minWhitePixelsLocation3) {
            RobotLogCommon.d(TAG, "The signal sleeve indicates location 3.");
            return new SignalSleeveReturn(RobotConstants.OpenCVResults.RECOGNITION_SUCCESSFUL, RobotConstantsPowerPlay.SignalSleeveLocation.THREE);
        }

        // Try location 2.
        RobotLogCommon.d(TAG, "Minimum non-zero-pixel count for location 2 " + minWhitePixelsLocation2);
        if (nonZeroPixelCount > minWhitePixelsLocation2) {
            RobotLogCommon.d(TAG, "The signal sleeve indicates location 2.");
            return new SignalSleeveReturn(RobotConstants.OpenCVResults.RECOGNITION_SUCCESSFUL, RobotConstantsPowerPlay.SignalSleeveLocation.TWO);
        }

        // Default: must be location 1.
        return new SignalSleeveReturn(RobotConstants.OpenCVResults.RECOGNITION_SUCCESSFUL, RobotConstantsPowerPlay.SignalSleeveLocation.ONE);
    }

}