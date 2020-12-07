package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.teamcode.auto.RobotConstants;

// Holds the results of image recognition.
public class RingReturn {

    public final boolean fatalComputerVisionError;
    public final RobotConstants.TargetZone targetZone;

    public RingReturn(boolean pFatalComputerVisionError, RobotConstants.TargetZone pTargetZone) {
        fatalComputerVisionError = pFatalComputerVisionError;
        targetZone = pTargetZone;
    }

}

