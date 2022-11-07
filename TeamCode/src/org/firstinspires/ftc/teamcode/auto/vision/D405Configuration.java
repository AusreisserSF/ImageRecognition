package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.teamcode.common.RobotConstantsPowerPlay;

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
    public final D405Camera camera1;
    public final D405Camera camera2;

    public D405Configuration(double pFieldOfView,
                             float pDepthScale,
                             D405Camera pCamera1, D405Camera pCamera2) {
        fieldOfView = pFieldOfView;
        depthScale = pDepthScale;
        camera1 = pCamera1;
        camera2 = pCamera2;
    }

    public static class D405Camera {
        public final RobotConstantsPowerPlay.D405Orientation orientation;
        public final String serialNumber;
        public final double distanceToCameraCanter;
        public final double offsetFromCameraCenter;

        public D405Camera(RobotConstantsPowerPlay.D405Orientation pOrientation,
                          String pSerialNumber,
                          double pDistanceToCameraCenter,
                          double pOffsetFromCameraCenter) {
            orientation = pOrientation;
            serialNumber = pSerialNumber;
            distanceToCameraCanter = pDistanceToCameraCenter;
            offsetFromCameraCenter = pOffsetFromCameraCenter;
        }
    }
}