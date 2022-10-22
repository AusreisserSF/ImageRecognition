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
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.time.LocalDateTime;
import java.util.ArrayList;

public class ConeStackRecognition {

    private static final String TAG = ConeStackRecognition.class.getSimpleName();

    private final String workingDirectory;
    private final ImageUtils imageUtils;
    private String outputFilenamePreamble;
    private Mat imageROI;
    private RobotConstants.Alliance alliance;

    public ConeStackRecognition() {
        workingDirectory = WorkingDirectory.getWorkingDirectory() + RobotConstants.imageDir;
        imageUtils = new ImageUtils();
    }

    // Returns the result of image analysis.
    // The targets are:
    // A stack of 1 to 5 red cones
    // A stack of 1 to 5 blue cones
    public void recognizeConeStack(ImageProvider pImageProvider,
                                   VisionParameters.ImageParameters pImageParameters,
                                   ConeStackParameters pConeStackParameters,
                                   RobotConstants.Alliance pAlliance,
                                   RobotConstantsPowerPlay.ConeStackRecognitionPath pConeStackRecognitionPath) throws InterruptedException {

        RobotLogCommon.d(TAG, "In ConeStackRecognition.recognizeConeStack");

        alliance = pAlliance;

        // LocalDateTime requires Android minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        Pair<Mat, LocalDateTime> coneStackImage = pImageProvider.getImage();
        if (coneStackImage == null)
            return; // new ConeStackReturn(RobotConstants.OpenCVResults.OCV_ERROR); // don't crash

        // The image may be RGB (from a camera) or BGR (OpenCV imread from a file).
        // OpenCV wants BGR; the possible conversion is taken care of in imageUtils.preProcessImage.
        Mat imgOriginal = coneStackImage.first.clone();

        String fileDate = TimeStamp.getLocalDateTimeStamp(coneStackImage.second);
        outputFilenamePreamble = imageUtils.createOutputFilePreamble(pImageParameters.image_source, workingDirectory, RobotConstants.imageFilePrefix, fileDate);
        imageROI = imageUtils.preProcessImage(pImageProvider, imgOriginal, outputFilenamePreamble, pImageParameters);

        RobotLogCommon.d(TAG, "Recognition path " + pConeStackRecognitionPath);
        // ConeStackReturn retVal;
        switch (pConeStackRecognitionPath) {
            case COLOR: {
                //retVal = colorRecognitionPath(pConeStackParameters.colorParameters);
                break;
            }
            case GRAYSCALE: {
                /*retVal = */
                grayRecognitionPath(pConeStackParameters);
                break;
            }
            default:
                throw new AutonomousRobotException(TAG, "Unsupported recognition path " + pConeStackRecognitionPath);
        }

        return; // retVal;
    }

    private void grayRecognitionPath(ConeStackParameters pConeStackParameters) {
        // Remove distractions before we convert to grayscale: depending on the
        // current alliance set the red or blue channel pixels to black.
        ArrayList<Mat> channels = new ArrayList<>(3);
        Core.split(imageROI, channels); // red or blue channel. B = 0, G = 1, R = 2

        // Write out the red channel as grayscale.
        Imgcodecs.imwrite(outputFilenamePreamble + "_RED.png", channels.get(2));
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_RED.png");

        Mat adjustedGray = imageUtils.adjustGrayscaleBrightness(channels.get(2), pConeStackParameters.redGrayscaleParameters.median_target);
        Imgcodecs.imwrite(outputFilenamePreamble + "_ADJ.png", adjustedGray);
        RobotLogCommon.d(TAG, "Writing adjusted grayscale image " + outputFilenamePreamble + "_ADJ.png");

        //**TODO Two paths to try: gray -> blur -> Canny -> contours and
        // gray -> blur -> threshold -> contours
        int grayThresholdLow = pConeStackParameters.redGrayscaleParameters.threshold_low;
        RobotLogCommon.d(TAG, "Threshold value: low " + grayThresholdLow);

        // Threshold the image: set pixels over the threshold value to white.
        Mat thresholded = new Mat(); // output binary image
        Imgproc.threshold(adjustedGray, thresholded,
                grayThresholdLow,    // threshold value
                255,   // white
                Imgproc.THRESH_BINARY); // thresholding type

        Imgcodecs.imwrite(outputFilenamePreamble + "_ADJ_THR.png", thresholded);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_ADJ_THR.png");
    }

}