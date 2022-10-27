package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.intellij.WorkingDirectory;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.opencv.core.Point;

import java.io.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class RealSenseUtils {

    // Read the depth file that corresponds to the color image file.
    // The file is a collection of bytes; read them into an array
    // and then convert to an array of shorts.
    public static short[] getDepthArrayFromFile(VisionParameters.ImageParameters pImageParameters) throws IOException {
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
        return depth16UC1;
    }

    public static Pair<Double, Double> getAngleToPixel(VisionParameters.ImageParameters pImageParameters,
                                                                  Point pTargetPixel) {
        // The coordinates of the target pixel are relative to the ROI.
        // We need to adjust those values to reflect the position of the
        // target pixel in the entire image.
        double targetPixelX = pImageParameters.image_roi.x + pTargetPixel.x;
        double targetPixelY = pImageParameters.image_roi.y + pTargetPixel.y;
        /*
               // Calculate the angle from the camera to the center of the bounding rectangle.
        // The centroid of the bounding rectangle is relative to the entire image, not
        // just the ROI.
        int objectCentroidX = pImageParameters.image_roi.x + largestBoundingRect.x + (largestBoundingRect.width / 2);
        int objectCentroidY = pImageParameters.image_roi.y + largestBoundingRect.y + (largestBoundingRect.height / 2);
        double angleFromCameraToObject = imageUtils.computeAngleToObjectCenter(pImageParameters.resolution_width, objectCentroidX, pRealSenseParameters.depthCameraFOV);
        RobotLogCommon.d(TAG, "Angle from camera to object " + angleFromCameraToObject);

        // Read the depth file that corresponds to the color image file.
        // The file is a collection of bytes; read them into an array
        // and then convert to an array of shorts.
        String filenameWithoutExt = pImageParameters.image_source.substring(0, pImageParameters.image_source.lastIndexOf('.'));
        String depthFilename = WorkingDirectory.getWorkingDirectory() + RobotConstants.imageDir + filenameWithoutExt + ".depth";
        byte[] depth8UC1 = new byte[pImageParameters.resolution_width * pImageParameters.resolution_height];
        try (InputStream output = new FileInputStream(depthFilename)) {
            try (DataInputStream depthInputStream =
                         new DataInputStream(output)) {
                int read = depthInputStream.read(depth8UC1);
            }
        }

        // Convert an array of bytes to an array of shorts.
        short[] depth16UC1 = new short[depth8UC1.length / 2]; // length is in bytes
        ByteBuffer.wrap(depth8UC1).order(ByteOrder.LITTLE_ENDIAN).asShortBuffer().get(depth16UC1);

        // Use the pixel position of the centroid of the object as an index
        // into the array of depth values and get the distance from the camera
        // to the centroid pixel. For example, for a 640 x 480 image --
        // row 0 is 0 .. 639
        // row 1 is 640 .. 1279
        // ...
        int column = objectCentroidX;
        int row = objectCentroidY * pImageParameters.resolution_width;
        int centroidPixelDepth = depth16UC1[column + row] &0xFFFF; // use as unsigned short
        float scaledPixelDepth = centroidPixelDepth * pRealSenseParameters.depthCameraScale;

        RobotLogCommon.d(TAG, "Distance from camera to pixel at x " + objectCentroidX + ", y " + objectCentroidY + " = " + scaledPixelDepth);

        // We have the hypotenuse from the camera to the center of the object
        // in meters and we have the angle from the camera to the center
        // of the object. Solve for the other two sides in meters.

        // sin(angleFromCameraToObjectr) = opposite / scaledPixelDepth
        double sinACC = Math.sin(Math.toRadians(angleFromCameraToObject));
        // The "opposite" side if the triangle is from the centroid of the
        // object to the center of the image.
        double opposite = sinACC * scaledPixelDepth;

        // cos(angleFromCameraToObject) = adjacent / scaledPixelDepth
        double cosACC = Math.cos(Math.toRadians(angleFromCameraToObject));
        // The "adjacent" side if the triangle is from the center of the
        // camera to the center of the image.
        double adjacentFromCameraCenter = cosACC * scaledPixelDepth;

        // Since the center of the robot is behind the camera, add this
        // distance to the adjacent value.
        double adjacentFromRobotCenter = adjacentFromCameraCenter + pRealSenseParameters.cameraToRobotCenterMeters;

        // We have a new triangle. We need to get the angle from the robot
        // center to the center of the object.
        double ratioARC = opposite / adjacentFromRobotCenter;
        double tanARC = Math.tan(ratioARC);
        double angleFromRobotCenter = Math.toDegrees(tanARC);
        RobotLogCommon.d(TAG, "Angle from robot center to object " + angleFromRobotCenter);

        // Get the distance from the center of the robot to the center of
        // the object. This is the hypotenuse of our new triangle.
        double distanceFromRobotCenter = Math.sqrt(Math.pow(opposite, 2) + Math.pow(adjacentFromRobotCenter, 2));
        RobotLogCommon.d(TAG, "Distance (meters) from robot center to pixel at x " + objectCentroidX + ", y " + objectCentroidY + " = " + distanceFromRobotCenter);

         */
    }
}
