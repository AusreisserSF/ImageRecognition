package org.firstinspires.ftc.teamcode.auto.vision;

//!! IntelliJ only

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
import java.util.List;
import java.util.Optional;

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
        grayRecognitionPath(pImageParameters, pConeStackParameters);
    }

    private ConeStackReturn grayRecognitionPath(VisionParameters.ImageParameters pImageParameters, ConeStackParameters pConeStackParameters) {
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
      }
        else if (alliance == RobotConstants.Alliance.BLUE) {
           grayMedianTarget = pConeStackParameters.blueGrayscaleParameters.median_target;
           // The threshold value will be negative, which indicates that we should use
            // THRESH_BINARY_INV.
           grayThresholdLow = pConeStackParameters.blueGrayscaleParameters.threshold_low;
      }

        Mat thresholded = imageUtils.performThresholdOnGray(channels.get(2), outputFilenamePreamble, grayMedianTarget, grayThresholdLow);

        // Identify the contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(thresholded, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        if (contours.size() == 0) {
            RobotLogCommon.d(TAG, "No contours found");
            return new ConeStackReturn(RobotConstants.OpenCVResults.RECOGNITION_UNSUCCESSFUL); // don't crash
        }

        Mat contoursDrawn = imageROI.clone();
        drawShapeContours(contours, contoursDrawn);
        Imgcodecs.imwrite(outputFilenamePreamble + "_CON.png", contoursDrawn);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_CON.png");

        // The largest contour should be the cone.
        Optional<MatOfPoint> largestContour = imageUtils.getLargestContour(contours);
        if (largestContour.isEmpty())
            return new ConeStackReturn(RobotConstants.OpenCVResults.RECOGNITION_UNSUCCESSFUL); // don't crash

        // Get the centroid of the largest contour.
        Point centroid = imageUtils.getContourCentroid(largestContour.get());
        RobotLogCommon.d(TAG, "Center of largest contour (cone): x " +
                centroid.x + ", y " + centroid.y);

        //**TODO Staying within the bounds of the contour, define a rectangle;
        // inside this rectangle we'll look for an in-range depth value.

        // Get the bounding box for the largest contour. Start at the center
        // of the bounding box and create a square or rectangle as the
        // window onto the contour. For each pixel in the rectangle first
        // test if it is both inside the contour (using pointPolygonTest())
        // and has a valid depth value. If so, get the depth and angle to
        // this pixel.

        // Get its bounding rectangle.
        Rect largestBoundingRect = Imgproc.boundingRect(largestContour.get());

        // Draw a rectangle around the largest contour.
        Mat drawnRectangle = imageROI.clone();
        drawOneRectangle(largestBoundingRect, drawnRectangle);
        Imgcodecs.imwrite(outputFilenamePreamble + "_BRECT.png", drawnRectangle);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_BRECT.png");

        // Calculate the angle from the camera to the center of the bounding rectangle.
        // The centroid of the bounding rectangle is relative to the entire image, not
        // just the ROI.
        int coneStackCentroidX = pImageParameters.image_roi.x + largestBoundingRect.x + (largestBoundingRect.width / 2);
        int coneStackCentroidY = pImageParameters.image_roi.y + largestBoundingRect.y + (largestBoundingRect.height / 2);


/*
// https://stackoverflow.com/questions/15721550/looping-through-opencv-mat-in-java-bindings
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(src, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Mat rawDist = new Mat(src.size(), CvType.CV_32F);
        float[] rawDistData = new float[(int) (rawDist.total() * rawDist.channels())];
        for (int i = 0; i < src.rows(); i++) {
            for (int j = 0; j < src.cols(); j++) {
                rawDistData[i * src.cols() + j] = (float) Imgproc
                        .pointPolygonTest(new MatOfPoint2f(contours.get(0).toArray()), new Point(j, i), true);
            }
        }
        rawDist.put(0, 0, rawDistData);

 */
        //**TODO create a kernel around the centroid, find a white pixel,
        // determine angle and distance from the depth file.

        return new ConeStackReturn(RobotConstants.OpenCVResults.RECOGNITION_SUCCESSFUL, 0, 0);
    }

    // The parameters pContours is the output of a call to findContours.
    private void drawShapeContours(List<MatOfPoint> pContours, Mat pImageOut) {
        RobotLogCommon.d(TAG, "drawContours: number of contours " + pContours.size());
        Scalar color = new Scalar(0, 255, 0); // BGR green - good against dark background

        for (int i = 0; i < pContours.size(); i++) {
            Imgproc.drawContours(pImageOut, pContours, i, color, 2);
        }
    }

    private void drawOneRectangle(Rect pRect, Mat pImageOut) {
        Imgproc.rectangle(pImageOut, pRect, new Scalar(0, 255, 0)); // GREEN
    }
}