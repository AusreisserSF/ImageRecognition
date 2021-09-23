package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.teamcode.common.RobotConstantsFreightFrenzy;

import java.util.ArrayList;
import java.util.Map;

// Input parameters to barcode recognition.
public class BarcodeParameters {

    public final VisionParameters.ImageParameters imageParameters; // extracted from BarcodeParameters.xml.
    public final VisionParameters.GrayParameters grayParameters; // extracted from BarcodeParameters.xml.
    private Map<RobotConstantsFreightFrenzy.BarcodeElementWithinROI, BarcodeElement> barcodeElements;
 
    public BarcodeParameters(VisionParameters.ImageParameters pImageParameters, VisionParameters.GrayParameters pGrayParameters) {
        imageParameters = pImageParameters;
        grayParameters = pGrayParameters;
    }

    public void setBarcodeElements(Map<RobotConstantsFreightFrenzy.BarcodeElementWithinROI, BarcodeElement> pBarcodeElements) {
        barcodeElements = pBarcodeElements;
    }

    public Map<RobotConstantsFreightFrenzy.BarcodeElementWithinROI, BarcodeElement> getBarcodeElements() {
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