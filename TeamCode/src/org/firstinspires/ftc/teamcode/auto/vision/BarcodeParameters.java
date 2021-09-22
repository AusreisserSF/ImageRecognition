package org.firstinspires.ftc.teamcode.auto.vision;

import java.util.ArrayList;

// Parameters extracted from BarcodeParameters.xml.
public class BarcodeParameters {

    public enum ShippingHubLevels {
        LEVEL_1(1), LEVEL_2(2), LEVEL_3(3);

        private int level;

        private ShippingHubLevels(int pLevel) {
            level = pLevel;
        }

        public int getLevel() {
            return level;
        }
    }

    public final VisionParameters.ImageParameters imageParameters;
    public final VisionParameters.GrayParameters grayParameters;
 
    public BarcodeParameters(VisionParameters.ImageParameters pImageParameters, VisionParameters.GrayParameters pGrayParameters) {
        imageParameters = pImageParameters;
        grayParameters = pGrayParameters;
    }

    // Each barcode element is associated with an x-coordinate relative to the
    // current region of interest (ROI), a width in pixels within which a
    // barcode element should be found, and a shipping hub level.
    public static class BarcodeElement {
        public final int x;
        public final int width;
        public final ShippingHubLevels shippingHubLevel;

        public BarcodeElement(int pX, int pWidth, ShippingHubLevels pShippingHubLevel) {
            x = pX;
            width = pWidth;
            shippingHubLevel = pShippingHubLevel;
        }
    }
    
}