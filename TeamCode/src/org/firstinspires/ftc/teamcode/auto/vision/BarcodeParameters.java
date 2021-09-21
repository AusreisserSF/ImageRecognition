package org.firstinspires.ftc.teamcode.auto.vision;

import java.util.ArrayList;

// Parameters extracted from BarcodeParameters.xml.
public class BarcodeParameters {

    public final VisionParameters.ImageParameters imageParameters;
    public final VisionParameters.GrayParameters grayParameters;
    public final ArrayList<BarcodeElement> barcodeElements;
 
    public BarcodeParameters(VisionParameters.ImageParameters pImageParameters, VisionParameters.GrayParameters pGrayParameters,
                          ArrayList<BarcodeElement> pBarcodeElements) {
        imageParameters = pImageParameters;
        grayParameters = pGrayParameters;
        barcodeElements = pBarcodeElements;
    }

    public static class BarcodeElement {
        public final int x;
        public final int width;

        public BarcodeElement(int pX, int pWidth) {
            x = pX;
            width = pWidth;
        }
    }
    
}