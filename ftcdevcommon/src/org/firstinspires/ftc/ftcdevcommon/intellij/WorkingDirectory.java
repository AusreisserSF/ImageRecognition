package org.firstinspires.ftc.ftcdevcommon.intellij;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

public class WorkingDirectory {

    private static final String TAG = "CommonUtils";
    private static final String filesPath = "Files";

    public static String getWorkingDirectory() {
        Path currentWorkingDirPath = Paths.get("");
        String currentPath = currentWorkingDirPath.toAbsolutePath().toString() + File.separator + filesPath;
        if (!Files.exists(Paths.get(currentPath)))
            throw new AutonomousRobotException(TAG, "Could not open working directory " + currentPath);

        return filesPath;
    }
}