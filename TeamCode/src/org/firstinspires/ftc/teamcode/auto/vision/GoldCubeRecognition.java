package org.firstinspires.ftc.teamcode.auto.vision;

//!! IntelliJ only

import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.intellij.TimeStamp;
import org.firstinspires.ftc.ftcdevcommon.intellij.WorkingDirectory;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.DataInputStream;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.time.LocalDateTime;
import java.util.List;
import java.util.Optional;

public class GoldCubeRecognition {

    private static final String TAG = GoldCubeRecognition.class.getSimpleName();

    private final String workingDirectory;
    private final ImageUtils imageUtils;

    public GoldCubeRecognition() {
        workingDirectory = WorkingDirectory.getWorkingDirectory() + RobotConstants.imageDir;
        imageUtils = new ImageUtils();
    }

    // Returns the result of image analysis.
    public GoldCubeReturn getAngleAndDistanceToGoldCube(ImageProvider pImageProvider,
                                                              VisionParameters.ImageParameters pImageParameters,
                                                              GoldCubeParameters pGoldCubeParameters) throws InterruptedException, IOException {

        RobotLogCommon.d(TAG, "In GoldCubeRecognition.getAngleAndDistanceToGoldCube");

        // LocalDateTime requires Android minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        Pair<Mat, LocalDateTime> goldCubeImage = pImageProvider.getImage();
        if (goldCubeImage.first == null)
            return new GoldCubeReturn(RobotConstants.OpenCVResults.OCV_ERROR); // don't crash

        // The image may be RGB (from a camera) or BGR (OpenCV imread from a file).
        Mat imgOriginal = goldCubeImage.first.clone();

        String fileDate = TimeStamp.getLocalDateTimeStamp(goldCubeImage.second);
        String outputFilenamePreamble = imageUtils.createOutputFilePreamble(pImageParameters.ocv_image, workingDirectory, RobotConstants.imageFilePrefix, fileDate);
        Mat imageROI = imageUtils.preProcessImage(pImageProvider, imgOriginal, outputFilenamePreamble, pImageParameters);

        List<MatOfPoint> contours = imageUtils.applyInRangeAndFindContours(imageROI, outputFilenamePreamble, pGoldCubeParameters.hsvParameters);
        if (contours.size() == 0) {
            RobotLogCommon.d(TAG, "No contours found");
            return new GoldCubeReturn(RobotConstants.OpenCVResults.RECOGNITION_UNSUCCESSFUL); // don't crash
        }

        Mat contoursDrawn = imageROI.clone();
        drawShapeContours(contours, contoursDrawn);
        Imgcodecs.imwrite(outputFilenamePreamble + "_CON.png", contoursDrawn);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_CON.png");

        // The largest contour should be the gold cube.
        Optional<MatOfPoint> largestContour = imageUtils.getLargestContour(contours);
        if (largestContour.isEmpty())
            return new GoldCubeReturn(RobotConstants.OpenCVResults.RECOGNITION_UNSUCCESSFUL); // don't crash

        // Get its bounding rectangle.
        Rect largestBoundingRect = Imgproc.boundingRect(largestContour.get());

        // Because the shaft is the same color as the level 2 platter the bounding
        // rectangle will be too high. But we only care about its width.
        RobotLogCommon.d(TAG, "Width of largest contour " + largestBoundingRect.width);

        // Draw a rectangle around the largest contour.
        Mat drawnRectangle = imageROI.clone();
        drawOneRectangle(largestBoundingRect, drawnRectangle);
        Imgcodecs.imwrite(outputFilenamePreamble + "_BRECT.png", drawnRectangle);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_BRECT.png");

        // Calculate the angle from the camera to the center of the bounding rectangle.
        // The centroid of the bounding rectangle is relative to the entire image, not
        // just the ROI.
        int goldCubeCentroidX = pImageParameters.image_roi.x + largestBoundingRect.x + (largestBoundingRect.width / 2);
        int goldCubeCentroidY = pImageParameters.image_roi.y + largestBoundingRect.y + (largestBoundingRect.height / 2);
        double angleFromCameraToGoldCube = imageUtils.computeAngleToObjectCenter(pImageParameters.resolution_width, goldCubeCentroidX, pGoldCubeParameters.colorCameraFOV);
        RobotLogCommon.d(TAG, "Angle from camera to gold cube " + angleFromCameraToGoldCube);

        // Read the depth file that corresponds to the color image file.
        String depthFilename = WorkingDirectory.getWorkingDirectory() + RobotConstants.imageDir + ".depth";
        int[] depth16UC1 = new int[pImageParameters.resolution_width * pImageParameters.resolution_height];
        try (InputStream output = new FileInputStream(depthFilename)) {
            try (DataInputStream depthInputStream =
                         new DataInputStream(output)) {
                for (int i = 0; i < depth16UC1.length; i++)
                    depth16UC1[i] = depthInputStream.readUnsignedShort();
            }
        }

        //**TODO Use the pixel position of the centroid of the gold cube as an index
        // into the array of depth values and get the distance from the camera to the
        // centroid pixel.
        // 0 .. 639
        // 640 .. 1279
        int column = goldCubeCentroidX;
        int row = goldCubeCentroidY * pImageParameters.resolution_width;
        int centroidPixelDepth = depth16UC1[column + row];
        float scaledPixelDepth = centroidPixelDepth * pGoldCubeParameters.depthCameraScale;

        RobotLogCommon.d(TAG, "Distance to pixel at x " + goldCubeCentroidX + ", y " + goldCubeCentroidY + " = " + scaledPixelDepth);

        //**TODO Now we have the hypotenuse from the camera to the center of the gold
        // cube in meters and we have the angle from the camera to the center of the
        // gold cube. Solve for the other two sides in meters.

        double angleFromRobotCenter = 0.0f;
        double distanceFromRobotCenter = 0.0f;
        return new GoldCubeReturn(RobotConstants.OpenCVResults.RECOGNITION_SUCCESSFUL, angleFromRobotCenter, distanceFromRobotCenter);
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