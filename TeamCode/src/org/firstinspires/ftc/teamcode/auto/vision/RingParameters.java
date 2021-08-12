package org.firstinspires.ftc.teamcode.auto.vision;

// Parameters extracted from RingParameters.xml.
public class RingParameters {

    public final VisionParameters.ImageParameters imageParameters;
    public final VisionParameters.HSVParameters hsvParameters;
    public final double minimum_pixel_count_1_ring;
    public final double minimum_pixel_count_4_rings;

    public RingParameters(VisionParameters.ImageParameters pImageParameters, VisionParameters.HSVParameters pHSVParameters,
                          double pMinPixelCount1Ring, double pMinPixelCount4Rings) {
        imageParameters = pImageParameters;
        hsvParameters = pHSVParameters;
        minimum_pixel_count_1_ring = pMinPixelCount1Ring;
        minimum_pixel_count_4_rings = pMinPixelCount4Rings;
    }
}