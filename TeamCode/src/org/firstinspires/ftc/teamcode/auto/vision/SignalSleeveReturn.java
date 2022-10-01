package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsPowerPlay;

// Holds the results of image recognition.
public class SignalSleeveReturn {

    public final RobotConstants.OpenCVResults openCVResults;
    public final RobotConstantsPowerPlay.SignalSleeveLocation signalSleeveLocation;

    public SignalSleeveReturn(RobotConstants.OpenCVResults pOpenCVResults, RobotConstantsPowerPlay.SignalSleeveLocation pSignalSleeveLocation) {
        openCVResults = pOpenCVResults;
        signalSleeveLocation = pSignalSleeveLocation;
    }

    // Constructor for OpenCV errors such as "no such file".
    public SignalSleeveReturn(RobotConstants.OpenCVResults pOpenCVResults) {
        openCVResults = pOpenCVResults;
        signalSleeveLocation = RobotConstantsPowerPlay.SignalSleeveLocation.SIGNAL_SLEEVE_LOCATION_NPOS;
    }
}