package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.teamcode.common.RobotConstantsPowerPlay;

import java.util.EnumMap;

/*
<DEPTH_CAMERA_D405 configured="yes">
    <characteristics>
      <field_of_view>87.0</field_of_view>
      <depth_scale>.0001</depth_scale>
    </characteristics>
    <camera_1 >
      <orientation>back</orientation>
      <serial_number>123622270958</serial_number>
      <distance_to_camera_canter>8.0</distance_to_camera_canter>
      <offset_from_camera_center>0.0</offset_from_camera_center>
    </camera_1>
    <!-- camera_2>
      <orientation>back</orientation>
      <serial_number>123622270236</serial_number>
      <distance_to_camera_canter></distance_to_camera_canter>
      <offset_from_camera_center></offset_from_camera_center>
    </camera_2 -->
  </DEPTH_CAMERA_D405>
 */
// Input parameters for junction recognition.
public class D405Configuration {

    public final double fieldOfView;
    public final float depthScale;
    public final EnumMap<RobotConstantsPowerPlay.D405CameraId, D405Camera> cameraMap =
            new EnumMap<>(RobotConstantsPowerPlay.D405CameraId.class);

    public D405Configuration(double pFieldOfView,
                             float pDepthScale,
                             D405Camera pCamera1, D405Camera pCamera2) {
        fieldOfView = pFieldOfView;
        depthScale = pDepthScale;
        cameraMap.put(pCamera1.orientation, pCamera1);
        if (pCamera2 != null)
            cameraMap.put(pCamera2.orientation, pCamera2);
    }

    public static class D405Camera {
        public final RobotConstantsPowerPlay.D405CameraId orientation;
        public final String serialNumber;
        public final double distanceToCameraCanter;
        public final double offsetFromCameraCenter;

        public D405Camera(RobotConstantsPowerPlay.D405CameraId pOrientation,
                          String pSerialNumber,
                          double pDistanceToCameraCenter,
                          double pOffsetFromCameraCenter) {
            orientation = pOrientation;
            serialNumber = pSerialNumber;

            // Keep these values in inches as they are given in the XML file.
            // pOffsetFromCameraCenter is positive if the camera is positioned
            // for the left of the robot's center as seen from behind.
            distanceToCameraCanter = pDistanceToCameraCenter;
            offsetFromCameraCenter = pOffsetFromCameraCenter;
        }
    }
}