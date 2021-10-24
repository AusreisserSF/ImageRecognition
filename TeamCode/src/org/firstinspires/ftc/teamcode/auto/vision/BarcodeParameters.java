package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.teamcode.common.RobotConstantsFreightFrenzy;

import java.util.EnumMap;

// Input parameters to barcode recognition.
public class BarcodeParameters {

    public final VisionParameters.GrayParameters grayParameters; // extracted from BarcodeParameters.xml.
    private EnumMap<RobotConstantsFreightFrenzy.BarcodeElementWithinROI, BarcodeElement> barcodeElements;
 
    public BarcodeParameters(VisionParameters.GrayParameters pGrayParameters) {
        grayParameters = pGrayParameters;
    }

    public void setBarcodeElements(EnumMap<RobotConstantsFreightFrenzy.BarcodeElementWithinROI, BarcodeElement> pBarcodeElements) {
        barcodeElements = pBarcodeElements;
    }

    public EnumMap<RobotConstantsFreightFrenzy.BarcodeElementWithinROI, BarcodeElement> getBarcodeElements() {
        return barcodeElements;
    }

    // Each barcode element is associated with an x-coordinate relative to
    // the current region of interest (ROI) and width in pixels within which
    // a barcode element should be found
    public static class BarcodeElement {

        public final int x;
        public final int width;

        public BarcodeElement(int pX, int pWidth) {
            x = pX;
            width = pWidth;
        }
    }
    
}