package org.firstinspires.ftc.teamcode.auto.vision;

// Parameters extracted from RingParameters.xml.
public class TowerParameters {

    public final CommonParameters.ImageParameters imageParameters;
    public final CommonParameters.GrayParameters grayParameters;
    public final double minimum_area;
    public final double maximum_area;
    public final double vumark_center_to_frame_center_angle;

    public TowerParameters(CommonParameters.ImageParameters pImageParameters,
                           CommonParameters.GrayParameters pGrayParameters,
                           double pMinArea, double pMaxArea,
                           double pAngle) {
        imageParameters = pImageParameters;
        grayParameters = pGrayParameters;
        minimum_area = pMinArea;
        maximum_area = pMaxArea;
        vumark_center_to_frame_center_angle = pAngle;
    }
}