package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.intellij.WorkingDirectory;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsPowerPlay;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.DataInputStream;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.*;
import java.util.stream.Collectors;

public class RealSenseUtils {

    private static final String TAG = RealSenseUtils.class.getSimpleName();
    private static final double INCHES_PER_METER = 39.37;

    // Read the depth file that corresponds to the color image file.
    // The file is a collection of bytes; read them into an array
    // and then convert to an array of shorts.
    public static short[] getDepthArrayFromFile(VisionParameters.ImageParameters pImageParameters) throws IOException {
        String filenameWithoutExt = pImageParameters.image_source.substring(0, pImageParameters.image_source.lastIndexOf('.'));
        String depthFilename = WorkingDirectory.getWorkingDirectory() + RobotConstants.imageDir + filenameWithoutExt + ".depth";
        byte[] depth8UC1 = new byte[pImageParameters.resolution_width * pImageParameters.resolution_height * 2]; // 16 bits per depth location
        try (InputStream output = new FileInputStream(depthFilename)) {
            try (DataInputStream depthInputStream =
                         new DataInputStream(output)) {
                depthInputStream.read(depth8UC1);
            }
        }

        // Convert an array of bytes to an array of shorts.
        short[] depth16UC1 = new short[depth8UC1.length / 2]; // length is in bytes
        ByteBuffer.wrap(depth8UC1).order(ByteOrder.LITTLE_ENDIAN).asShortBuffer().get(depth16UC1);
        return depth16UC1;
    }

    // Adapted from the RealSense example RealsenseAlignAdv.
    // Apply background removal, converting to gray those pixels in the image
    // ROI that are less than the minimum distance parameter or greater than
    // the maximum distance parameter.
    // Note: the depth frame has the same dimensions as the full image.
    public static Mat removeBackground(Mat pImageROI, VisionParameters.ImageParameters pImageParameters,
                                       D405Configuration pD405Configuration,
                                       short[] pDepth16UC1,
                                       double pMinDistance, double pMaxDistance) {

        // Follow the RealsenseAlignAdv example; we need to convert the image ROI
        // to an array of bytes as shown here ---
        // https://stackoverflow.com/questions/27065062/opencv-mat-object-serialization-in-java
        byte[] imageROIBytes = new byte[(int) (pImageROI.total() * pImageROI.elemSize())];
        pImageROI.get(0, 0, imageROIBytes);
        int imageROIBytesIndex = 0;

        int roiOriginX = pImageParameters.image_roi.x;
        int roiEndX = roiOriginX + pImageParameters.image_roi.width;
        int roiOriginY = pImageParameters.image_roi.y;
        int roiEndY = roiOriginY + pImageParameters.image_roi.height;
        int depthPixelRowIndex;
        double pixelDistance;
        int pixelsGrayedOut = 0;
        int pixelsInRange = 0;

        // Iterate through the depth data but only check those locations
        // that correspond to the image ROI.
        for (int i = roiOriginY; i < roiEndY; i++) {
            depthPixelRowIndex = i * pImageParameters.resolution_width; // the start of each row
            for (int j = roiOriginX; j < roiEndX; j++) {
                pixelDistance = pD405Configuration.depthScale * pDepth16UC1[depthPixelRowIndex + j];
                // Check if the depth value is less or greater than the threshold.
                if (pixelDistance <= pMinDistance || pixelDistance > pMaxDistance) {
                    // Replace the BGR bytes in the copy of the input image with gray.
                    imageROIBytes[imageROIBytesIndex] = (byte) 0x60; // b
                    imageROIBytes[imageROIBytesIndex + 1] = (byte) 0x60; // g
                    imageROIBytes[imageROIBytesIndex + 2] = (byte) 0x60; // r
                    pixelsGrayedOut++;
                } else
                    pixelsInRange++;
                imageROIBytesIndex += 3;
            }
        }

        RobotLogCommon.d(TAG, "Pixels in depth range " + pixelsInRange);
        RobotLogCommon.d(TAG, "Pixels out of depth range " + pixelsGrayedOut);

        // We need to put the image ROI in the form of array of bytes back into
        // an OpenCV Mat.
        // Again from https://stackoverflow.com/questions/27065062/opencv-mat-object-serialization-in-java
        Mat depthAdjustedROI = new Mat(pImageParameters.image_roi.height, pImageParameters.image_roi.width, CvType.CV_8UC3);
        depthAdjustedROI.put(0, 0, imageROIBytes);
        return depthAdjustedROI;
    }

    // pImageROI is cropped version of the original ROI.
    // pThresholded is the thresholded output of pImageROI.
    public static RealSenseReturn getAngleAndDistance(Mat pImageROI, Mat pThresholded,
                                                      D405Configuration pD405Configuration,
                                                      RobotConstantsPowerPlay.D405CameraId pCameraId,
                                                      short[] pDepthArray,
                                                      String pOutputFilenamePreamble,
                                                      VisionParameters.ImageParameters pImageParameters,
                                                      DepthParameters pDepthParameters) {
        // Identify the contours.
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(pThresholded, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        if (contours.size() == 0) {
            RobotLogCommon.d(TAG, "No contours found");
            return new RealSenseReturn(RobotConstants.RecognitionResults.RECOGNITION_UNSUCCESSFUL); // don't crash
        }

        // Within the ROI draw all of the contours.
        Mat contoursDrawn = pImageROI.clone();
        drawShapeContours(contours, contoursDrawn);
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_CON.png", contoursDrawn);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_CON.png");

        // The largest contour should be the cone.
        Optional<MatOfPoint> largestContour = ImageUtils.getLargestContour(contours);
        if (largestContour.isEmpty()) {
            RobotLogCommon.d(TAG, "Largest contour not found");
            return new RealSenseReturn(RobotConstants.RecognitionResults.RECOGNITION_UNSUCCESSFUL); // don't crash
        }

        //## Because the depth array from the camera covers the entire image
        // always log full-image coordinates.

        // Define a bounding rectangle for the largest contour.
        Rect largestBoundingRect = Imgproc.boundingRect(largestContour.get());
        RobotLogCommon.d(TAG, "Largest bounding rectangle in full image: x " +
                (largestBoundingRect.x + pImageParameters.image_roi.x) +
                ", y " + (largestBoundingRect.y + pImageParameters.image_roi.y) +
                ", width " + largestBoundingRect.width + ", height " + largestBoundingRect.height);

        // Within the ROI draw a rectangle around the largest contour.
        Mat drawnRectangle = pImageROI.clone();
        drawOneRectangle(largestBoundingRect, drawnRectangle, 2);
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_BRECT.png", drawnRectangle);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_BRECT.png");

        // We want to define a search rectangle, the x-coordinate of which
        // at its center is the same as that of the bounding box.
        int pixelSearchBoxCenter = largestBoundingRect.width / 2;

        // Subtract a percentage to get the left edge of the search box.
        double percentageOfWidth = largestBoundingRect.width * (pDepthParameters.depthWindowOffsetX / 100.0);
        int pixelSearchX = (int) ((largestBoundingRect.x + pixelSearchBoxCenter) - percentageOfWidth);
        double pixelSearchWidth = largestBoundingRect.width * (pDepthParameters.depthWindowWidth / 100.0);

        // Place the y-origin of the pixel search box at a reasonable distance
        // from the bottom of the bounding box.
        double percentageOfHeight = largestBoundingRect.height * (pDepthParameters.depthWindowOffsetY / 100.0);
        int pixelSearchY = (int) ((largestBoundingRect.y + largestBoundingRect.height) - percentageOfHeight);
        double pixelSearchHeight = largestBoundingRect.height * (pDepthParameters.depthWindowHeight / 100.0);
        RobotLogCommon.d(TAG, "Pixel search box in full image: x " +
                (pixelSearchX + pImageParameters.image_roi.x) +
                ", y " + (pixelSearchY + pImageParameters.image_roi.y) +
                ", width " + pixelSearchWidth + ", height " + pixelSearchHeight + ", area " + (pixelSearchWidth * pixelSearchHeight));

        // Sanity check.
        if (pixelSearchWidth == 0 || pixelSearchHeight == 0) {
            RobotLogCommon.d(TAG, "Pixel search box width or height is 0");
            return new RealSenseReturn(RobotConstants.RecognitionResults.RECOGNITION_UNSUCCESSFUL); // don't crash
        }

        // Draw the pixel search box on the ROI.
        Rect pixelSearchRect = new Rect(pixelSearchX, pixelSearchY, (int) pixelSearchWidth, (int) pixelSearchHeight);
        drawOneRectangle(pixelSearchRect, drawnRectangle, -1);
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_PRECT.png", drawnRectangle);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_PRECT.png");

        // Make sure the pixel search box (top left and bottom right) is
        // within the boundaries of the bounding box of the largest contour.
        if (!(largestBoundingRect.contains(pixelSearchRect.tl()) &&
                largestBoundingRect.contains(pixelSearchRect.br()))) {
            RobotLogCommon.d(TAG, "Pixel search box top out of range");
            return new RealSenseReturn(RobotConstants.RecognitionResults.RECOGNITION_UNSUCCESSFUL);
        }

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
        int targetPixelX, targetPixelY, targetPixelRow;
        float scaledPixelDepth;

        // Loop through all the pixels in the pixel search box and save the
        // depth and location of in-range pixels *in the full image* for
        // later sorting. To do this we'll need a method-local inner class.
        class InRangePixel {
            public final int fullImageX;
            public final int fullImageY;
            private final float scaledDepth;

            public InRangePixel(int pFullImageX, int pFullImageY, float pScaledDepth) {
                fullImageX = pFullImageX;
                fullImageY = pFullImageY;
                scaledDepth = pScaledDepth;
            }

            public float getScaledDepth() {
                return scaledDepth;
            }
        }

        List<InRangePixel> inRangePixels = new ArrayList<>();
        for (int i = pixelSearchY; i < pixelSearchY + pixelSearchHeight; i++) { // row
            for (int j = pixelSearchX; j < pixelSearchX + pixelSearchWidth; j++) { // column
                testReturn = (float) Imgproc
                        .pointPolygonTest(largestContourPoint2f, new Point(j, i), false);
                if (testReturn == 0.0 || testReturn == 1.0) {
                    // The depth array has values for every pixel in the full
                    // image, not just the ROI. So to test a pixel for depth
                    // we need to get its position in the full image.
                    targetPixelX = pImageParameters.image_roi.x + j;
                    targetPixelY = pImageParameters.image_roi.y + i;

                    // Use the pixel position of the centroid of the object as an index
                    // into the array of depth values and get the distance from the camera
                    // to the centroid pixel. For example, for a 640 x 480 image --
                    // row 0 is 0 .. 639
                    // row 1 is 640 .. 1279
                    // ...
                    targetPixelRow = targetPixelY * pImageParameters.resolution_width;
                    int centroidPixelDepth = pDepthArray[targetPixelRow + targetPixelX] & 0xFFFF; // use as unsigned short
                    scaledPixelDepth = centroidPixelDepth * pD405Configuration.depthScale;

                    // If the depth of the pixel is in range store its coordinates in
                    // the full image and its depth.
                    if (scaledPixelDepth >= pDepthParameters.minDepth &&
                            scaledPixelDepth <= pDepthParameters.maxDepth)
                        inRangePixels.add(new InRangePixel(targetPixelX, targetPixelY, scaledPixelDepth)); // yes, save it
                }
            }
        }

        // Make sure we've found at least one pixel.
        if (inRangePixels.size() == 0) {
            RobotLogCommon.d(TAG, "Did not find a pixel on or inside the cone contour");
            return new RealSenseReturn(RobotConstants.RecognitionResults.RECOGNITION_UNSUCCESSFUL);
        }

        // Sort the in-range pixels by their depth.
        RobotLogCommon.d(TAG, "Found " + inRangePixels.size() + " in-range pixels");
        List<InRangePixel> sortedInRangePixels = inRangePixels.stream()
                .sorted(Comparator.comparing(InRangePixel::getScaledDepth)) // ascending
                .collect(Collectors.toList());

        InRangePixel closestPixel = sortedInRangePixels.get(0);
        InRangePixel furthestPixel = sortedInRangePixels.get(sortedInRangePixels.size() - 1);
        RobotLogCommon.d(TAG, "Closest in-range pixel in the full image at: x " +
                closestPixel.fullImageX +
                ", y " + closestPixel.fullImageY + ", depth " + closestPixel.getScaledDepth());
        RobotLogCommon.d(TAG, "Furthest in-range pixel in the full image at: x " +
                furthestPixel.fullImageX +
                ", y " + furthestPixel.fullImageY + ", depth " + furthestPixel.getScaledDepth());

        // Now we want to average the coordinates and depth of the closest 50 (max) pixels.
        InRangePixel inRangePixel;
        int accumulatedFullImageX = 0;
        int accumulatedFullImageY = 0;
        float accumulatedDepth = 0.0f;
        int i;
        for (i = 0; i < 50 && i < sortedInRangePixels.size(); i++) {
            inRangePixel = sortedInRangePixels.get(i);
            accumulatedFullImageX += inRangePixel.fullImageX;
            accumulatedFullImageY += inRangePixel.fullImageY;
            accumulatedDepth += inRangePixel.scaledDepth;
        }

        int averageFullImageX = accumulatedFullImageX / i;
        int averageFullImageY = accumulatedFullImageY / i;
        float averageDepth = accumulatedDepth / i;
        RobotLogCommon.d(TAG, "Average x, y, depth of the closest 50 (max) pixels in the full image " +
                averageFullImageX + ", " + averageFullImageY + ", " + averageDepth);

        Pair<Double, Double> angleAndDistanceToPixel = getAngleAndDistanceToPixel(pD405Configuration, pCameraId,
                pImageParameters,
                averageFullImageX, averageFullImageY, averageDepth);
        return new RealSenseReturn(RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL, angleAndDistanceToPixel.first, angleAndDistanceToPixel.second);
    }

    // Returns the angle and distance from the center of the robot to
    // the target pixel.
    //##!! The coordinates of the target pixel are relative to the full image.
    private static Pair<Double, Double> getAngleAndDistanceToPixel(D405Configuration pD405Configuration, RobotConstantsPowerPlay.D405CameraId pCameraId,
                                                                   VisionParameters.ImageParameters pImageParameters,
                                                                   int pTargetPixelX, int pTargetPixelY, double pScaledPixelDepth) {
        // Calculate the angle from the camera to the target pixel.
        double angleFromCameraToPixel = ImageUtils.computeAngleToObjectCenter(pImageParameters.resolution_width, pTargetPixelX, pD405Configuration.fieldOfView);
        RobotLogCommon.d(TAG, "Angle from camera to target pixel (degrees) " + angleFromCameraToPixel);

        // Get the angle from the center of the robot to the target pixel.
        D405Configuration.D405Camera cameraData = pD405Configuration.cameraMap.get(pCameraId);
        double distanceFromCameraToPixel = pScaledPixelDepth * INCHES_PER_METER;
        RobotLogCommon.d(TAG, "Distance from camera to target pixel (inches) " + distanceFromCameraToPixel);

        double correctedAngle =
                CameraToCenterCorrections.getCorrectedAngle(Objects.requireNonNull(cameraData).distanceToCameraCanter,
                        cameraData.offsetFromCameraCenter, distanceFromCameraToPixel, angleFromCameraToPixel);

        RobotLogCommon.d(TAG, "Angle from robot center to target pixel (degrees) " + correctedAngle);

        // Get the distance from the center of the robot to the target pixel.
        double correctedDistance = CameraToCenterCorrections.getCorrectedDistance(cameraData.distanceToCameraCanter,
                cameraData.offsetFromCameraCenter, distanceFromCameraToPixel, angleFromCameraToPixel);
        RobotLogCommon.d(TAG, "Distance (inches) from robot center to pixel in full image at x " + pTargetPixelX + ", y " + pTargetPixelY + " = " + correctedDistance);

        return Pair.create(correctedAngle, correctedDistance);
    }

    // The parameters pContours is the output of a call to findContours.
    private static void drawShapeContours(List<MatOfPoint> pContours, Mat pImageOut) {
        RobotLogCommon.d(TAG, "drawContours: number of contours " + pContours.size());
        Scalar color = new Scalar(0, 255, 0); // BGR green - good against dark background

        for (int i = 0; i < pContours.size(); i++) {
            Imgproc.drawContours(pImageOut, pContours, i, color, 2);
        }
    }

    // Thickness of < 0 means fill with color.
    private static void drawOneRectangle(Rect pRect, Mat pImageOut, int pThickness) {
        Imgproc.rectangle(pImageOut, pRect, new Scalar(0, 255, 0), pThickness); // GREEN
    }
}
