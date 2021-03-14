package org.firstinspires.ftc.teamcode.auto;

import javafx.application.Application;
import javafx.scene.Scene;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import javafx.scene.text.FontWeight;
import javafx.scene.text.Text;
import javafx.stage.Stage;
import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.intellij.WorkingDirectory;
import org.firstinspires.ftc.teamcode.auto.vision.*;
import org.firstinspires.ftc.teamcode.auto.xml.RingParametersXML;
import org.firstinspires.ftc.teamcode.auto.xml.TowerParametersXML;
import org.opencv.core.Core;

import java.io.FileInputStream;
import java.io.InputStream;

public class RecognitionDispatcher extends Application {

    private static final String TAG = "RingRecognition";

    public static final double FIELD_WIDTH = 410;
    public static final double FIELD_HEIGHT = 300;

    public static final double NOTIFICATION_TEXT_POSITION_Y = (FIELD_HEIGHT / 2) + 20;
    public static final double NOTIFICATION_TEXT_POSITION_X = 10;
    public static final Font NOTIFICATION_TEXT_FONT = Font.font("Comic Sans MS", FontWeight.BOLD, 24);

    // Load OpenCV.
    private static final boolean openCVInitialized;
    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME); // IntelliJ only
        openCVInitialized = true; // IntelliJ only
    }

    // See https://stackoverflow.com/questions/21729362/trying-to-call-a-javafx-application-from-java-nosuchmethodexception
    //!! Default constructor is required.
    public RecognitionDispatcher() {
        RobotLogCommon.initialize(WorkingDirectory.getWorkingDirectory() + RingRecognitionConstants.logDir);
        RobotLogCommon.i(TAG, "Starting ring recognition");

        if (!openCVInitialized)
            new AutonomousRobotException(TAG, "Failure in OpenCV initialization");
    }

    @Override
    public void start(Stage pStage) throws Exception {
        // Parent root = FXMLLoader.load(getClass().getResource("simulator.fxml"));
        Pane field = new Pane();

        // Read the parameters for ring recognition from the xml file.
        RingParametersXML ringParametersXML = new RingParametersXML(WorkingDirectory.getWorkingDirectory() + RingRecognitionConstants.xmlDir);
        RingParameters ringParameters = ringParametersXML.getRingParameters();
        RingRecognition ringRecognition = new RingRecognition();

        // Read the parameters for the alignment of the robot with the tower goal
        // from the xml file.
        TowerParametersXML towerParametersXML = new TowerParametersXML(WorkingDirectory.getWorkingDirectory() + RingRecognitionConstants.xmlDir);
        TowerParameters towerParameters = towerParametersXML.getTowerParameters();
        TowerGoalAlignment towerGoalAlignment = new TowerGoalAlignment();

        String imagePath = WorkingDirectory.getWorkingDirectory() + RingRecognitionConstants.imageDir;

        // Align the robot with the tower goal.
        FileImage towerGoalFileImage = new FileImage(imagePath + towerParameters.imageParameters.file_name);
        double alignmentAngle = towerGoalAlignment.getAngleToTowerGoal(towerGoalFileImage, towerParameters);
        System.out.println("Tower goal alignment angle " + alignmentAngle);
        System.exit(0);


        // Call the OpenCV subsystem.
        FileImage fileImage = new FileImage(imagePath + ringParameters.imageParameters.file_name);
        RingReturn ringReturn = ringRecognition.findGoldRings(fileImage, ringParameters);
        if (ringReturn.fatalComputerVisionError)
            throw new AutonomousRobotException(TAG, "Error in computer vision subsystem");

        RobotLogCommon.d(TAG, "Found Target Zone " + ringReturn.targetZone);

        // Display the image in the Pane.
        InputStream stream = new FileInputStream(imagePath + ringParameters.imageParameters.file_name);
        Image image = new Image(stream);
        ImageView imageView = new ImageView();
        imageView.setImage(image);
        imageView.setX(0);
        imageView.setY(0);
        imageView.setFitWidth(FIELD_WIDTH / 2);
        imageView.setFitHeight(FIELD_HEIGHT / 2);
        imageView.setPreserveRatio(true);
        field.getChildren().add(imageView);

        // Write text to field.
        Text ringText = new Text("Rings indicate " + ringReturn.targetZone);
        ringText.setFont(NOTIFICATION_TEXT_FONT);
        ringText.setFill(Color.CYAN);
        ringText.setX(NOTIFICATION_TEXT_POSITION_X);
        ringText.setY(NOTIFICATION_TEXT_POSITION_Y);
        field.getChildren().add(ringText);

        Scene scene = new Scene(field, FIELD_WIDTH, FIELD_HEIGHT, Color.GRAY);
        pStage.setTitle("FTC 2020 - 2021 Ultimate Goal");
        pStage.setScene(scene);
        pStage.show();

        RobotLogCommon.closeLog();
    }

}