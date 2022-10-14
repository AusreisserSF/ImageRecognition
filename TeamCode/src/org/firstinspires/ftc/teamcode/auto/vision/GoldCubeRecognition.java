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
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
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
        String outputFilenamePreamble = imageUtils.createOutputFilePreamble(pImageParameters.image_source, workingDirectory, RobotConstants.imageFilePrefix, fileDate);
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
        // The file is a collection of bytes; read them into an array
        // and then convert to an array of shorts.
        String filenameWithoutExt = pImageParameters.image_source.substring(0, pImageParameters.image_source.lastIndexOf('.'));
        String depthFilename = WorkingDirectory.getWorkingDirectory() + RobotConstants.imageDir + filenameWithoutExt + ".depth";
        byte[] depth8UC1 = new byte[pImageParameters.resolution_width * pImageParameters.resolution_height];
        try (InputStream output = new FileInputStream(depthFilename)) {
            try (DataInputStream depthInputStream =
                         new DataInputStream(output)) {
                depthInputStream.read(depth8UC1);
            }
        }

        // Convert an array of bytes to an array of shorts.
        short[] depth16UC1 = new short[depth8UC1.length / 2]; // length is in bytes
        ByteBuffer.wrap(depth8UC1).order(ByteOrder.LITTLE_ENDIAN).asShortBuffer().get(depth16UC1);

        // Use the pixel position of the centroid of the gold cube as an index
        // into the array of depth values and get the distance from the camera to the
        // centroid pixel. For example, for a 640 x 480 image --
        // row 0 is 0 .. 639
        // row 1 is 640 .. 1279
        // ...
        int column = goldCubeCentroidX;
        int row = goldCubeCentroidY * pImageParameters.resolution_width;
        int centroidPixelDepth = depth16UC1[column + row] &0xFFFF; // use as unsigned short
        float scaledPixelDepth = centroidPixelDepth * pGoldCubeParameters.depthCameraScale;

        RobotLogCommon.d(TAG, "Distance from camera to pixel at x " + goldCubeCentroidX + ", y " + goldCubeCentroidY + " = " + scaledPixelDepth);

        // We have the hypotenuse from the camera to the center of the gold
        // cube in meters and we have the angle from the camera to the center
        // of the gold cube. Solve for the other two sides in meters.

        // sin(angleFromCameraToGoldCube) = opposite / scaledPixelDepth
        double sinACC = Math.sin(Math.toRadians(angleFromCameraToGoldCube));
        // The "opposite" side if the triangle is from the centroid of the
        // gold cube to the center of the image.
        double opposite = sinACC * scaledPixelDepth;

        // cos(angleFromCameraToGoldCube) = adjacent / scaledPixelDepth
        double cosACC = Math.cos(Math.toRadians(angleFromCameraToGoldCube));
        // The "adjacent" side if the triangle is from the center of the
        // camera to the center of the image.
        double adjacentFromCameraCenter = cosACC * scaledPixelDepth;

        // Since the center of the robot is behind the camera, add this
        // distance to the adjacent value.
        double adjacentFromRobotCenter = adjacentFromCameraCenter + pGoldCubeParameters.cameraToRobotCenterMeters;

        // We have a new triangle. We need to get the angle from the robot
        // center to the center of the gold cube.
        double ratioARC = opposite / adjacentFromRobotCenter;
        double tanARC = Math.tan(ratioARC);
        double angleFromRobotCenter = Math.toDegrees(tanARC);
        RobotLogCommon.d(TAG, "Angle from robot center to gold cube " + angleFromRobotCenter);

        // Get the distance from the center of the robot to the center of
        // the gold cube. This is the hypotenuse of our new ttriangle.
        double distanceFromRobotCenter = Math.sqrt(Math.pow(opposite, 2) + Math.pow(adjacentFromRobotCenter, 2));
        RobotLogCommon.d(TAG, "Distance (meters) from robot center to pixel at x " + goldCubeCentroidX + ", y " + goldCubeCentroidY + " = " + distanceFromRobotCenter);

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