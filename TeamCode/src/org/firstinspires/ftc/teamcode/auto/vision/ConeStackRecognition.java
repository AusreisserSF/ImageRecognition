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

import java.io.IOException;
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
                                   RobotConstantsPowerPlay.ConeStackRecognitionPath pConeStackRecognitionPath) throws InterruptedException, IOException {

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

    private ConeStackReturn grayRecognitionPath(VisionParameters.ImageParameters pImageParameters, ConeStackParameters pConeStackParameters)
            throws IOException {
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

        // Define a bounding rectangle for the largest contour.
        Rect largestBoundingRect = Imgproc.boundingRect(largestContour.get());

        // Draw a rectangle around the largest contour.
        Mat drawnRectangle = imageROI.clone();
        drawOneRectangle(largestBoundingRect, drawnRectangle);
        Imgcodecs.imwrite(outputFilenamePreamble + "_BRECT.png", drawnRectangle);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_BRECT.png");

        // We'll need the depth data from the D405 below.
        short[] depthArray = RealSenseUtils.getDepthArrayFromFile(pImageParameters);

        // We want to define a search rectangle whose x dimension
        // at its center is the same as that of the bounding box.
        int pixelSearchBoxCenter = largestBoundingRect.width / 2;

        //**TODO test for reasonable dimensions

        // Subtract a reasonable value to get the left edge of
        // the search box.
        int pixelSearchX = (largestBoundingRect.x + pixelSearchBoxCenter) - pConeStackParameters.depthParameters.depthWindowOffsets.x;
        int pixelSearchWidth = pConeStackParameters.depthParameters.depthWindowOffsets.width;

        // Place the y-origin of the pixel search box at a
        // reasonable distance from the bottom of the bounding
        // box.
        int pixelSearchY = largestBoundingRect.height - pConeStackParameters.depthParameters.depthWindowOffsets.y;
        int pixelSearchHeight = pConeStackParameters.depthParameters.depthWindowOffsets.height;

        // The following is from the OpenCV documentation; we want the
        // option where measureDist=false.
        /*
         The function determines whether the point is inside a contour,
         outside, or lies on an edge (or coincides with a vertex). It
         returns positive (inside), negative (outside), or zero (on an
         edge) value, correspondingly. When measureDist=false, the
         return value is +1, -1, and 0, respectively. Otherwise, the
         return value is a signed distance between the point and the
         nearest contour edge.
        */
        // A MatOfPoint2f is required by pointPolygonTest but since
        // we're only looking at the largest contour we don't need
        // to loop through all contours and can do the conversion
        // once.
        MatOfPoint2f largestContourPoint2f = new MatOfPoint2f(largestContour.get().toArray());
        float testReturn;
        boolean foundPixel = false;
        int foundPixelX = 0, foundPixelY = 0;
        double scaledPixelDepth = 0;
        for (int i = pixelSearchY; i < pixelSearchY + pixelSearchHeight; i++) {
            for (int j = pixelSearchX; j < pixelSearchX + pixelSearchWidth; j++) {
                testReturn = (float) Imgproc
                        .pointPolygonTest(largestContourPoint2f, new Point(j, i), false);
                if (testReturn == 0.0 || testReturn == 1.0) {
                    //**TODO apply depth filters here ...
                    //!! The depth array reflects the entire image; the pixel
                    // coordinates are relative to the ROI. ADJUST
                    foundPixel = true;
                    foundPixelX = j;
                    foundPixelY = i;
                    break;
                }
            }

            if (foundPixel)
                break;
        }

        if (foundPixel)
            RobotLogCommon.d(TAG, "Found a pixel on or inside the cone contour at x " +
                    foundPixelX + ", y " + foundPixelY);
        else RobotLogCommon.d(TAG, "Did not find a pixel on or inside the cone contour");

        Pair<Double, Double> angleAndDistanceToPixel = RealSenseUtils.getAngleAndDistanceToPixel(pImageParameters,
                foundPixelX, foundPixelY, scaledPixelDepth);


        return new ConeStackReturn(RobotConstants.OpenCVResults.RECOGNITION_SUCCESSFUL, angleAndDistanceToPixel.first, angleAndDistanceToPixel.second);
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