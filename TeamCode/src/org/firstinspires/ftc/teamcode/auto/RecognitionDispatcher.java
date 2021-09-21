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
import org.firstinspires.ftc.ftcdevcommon.RobotXMLElement;
import org.firstinspires.ftc.ftcdevcommon.XPathAccess;
import org.firstinspires.ftc.ftcdevcommon.intellij.WorkingDirectory;
import org.firstinspires.ftc.teamcode.auto.vision.*;
import org.firstinspires.ftc.teamcode.auto.xml.RingParametersXML;
import org.firstinspires.ftc.teamcode.common.RobotActionXML;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.opencv.core.Core;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.List;

public class RecognitionDispatcher extends Application {

    private static final String TAG = "RingRecognition";

    public static final double FIELD_WIDTH = 410;
    public static final double FIELD_HEIGHT = 300;

    public static final double NOTIFICATION_TEXT_POSITION_Y = (FIELD_HEIGHT / 2) + 20;
    public static final double NOTIFICATION_TEXT_POSITION_X = 10;
    public static final Font NOTIFICATION_TEXT_FONT = Font.font("Comic Sans MS", FontWeight.BOLD, 24);

    private Stage stage;
    private Pane field;

    // Load OpenCV.
    private static final boolean openCVInitialized;

    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME); // IntelliJ only
        openCVInitialized = true; // IntelliJ only
    }

    // See https://stackoverflow.com/questions/21729362/trying-to-call-a-javafx-application-from-java-nosuchmethodexception
    //!! Default constructor is required.
    public RecognitionDispatcher() {
        RobotLogCommon.initialize(WorkingDirectory.getWorkingDirectory() + RobotConstants.logDir);
        RobotLogCommon.i(TAG, "Starting ring recognition");

        if (!openCVInitialized)
            throw new AutonomousRobotException(TAG, "Failure in OpenCV initialization");
    }

    @Override
    public void start(Stage pStage) throws Exception {
        stage = pStage;
        field = new Pane();

        // Use RobotAction.xml but for a single action only.
        RobotActionXML robotActionXML = new RobotActionXML(WorkingDirectory.getWorkingDirectory() + RobotConstants.xmlDir);
        RobotActionXML.RobotActionData actionData = robotActionXML.getOpModeData("TEST");
        List<RobotXMLElement> actions = actionData.actions;
        if (actions.size() != 1)
            throw new AutonomousRobotException(TAG, "TEST OpMode must contain a single action");

        // Set up XPath access to the current action command.
        RobotXMLElement actionElement = actions.get(0);
        XPathAccess commandXPath = new XPathAccess(actionElement);

        // The only allowed image_provider is "file".
        String imageProviderId = commandXPath.getStringInRange("ocv_image_provider", commandXPath.validRange("vuforia", "file"));
        if (!imageProviderId.equals("file"))
            throw new AutonomousRobotException(TAG, "image_provider must be 'file'");

        String commandName = actionElement.getRobotXMLElementName().toUpperCase();
        switch (commandName) {
            case "RECOGNIZE_RINGS": {

                // Read the parameters for ring recognition from the xml file.
                RingParametersXML ringParametersXML = new RingParametersXML(WorkingDirectory.getWorkingDirectory() + RobotConstants.xmlDir);
                RingParameters ringParameters = ringParametersXML.getRingParameters();
                RingRecognition ringRecognition = new RingRecognition();

                // Call the OpenCV subsystem.
                String imagePath = WorkingDirectory.getWorkingDirectory() + RobotConstants.imageDir;
                ImageProvider fileImage = new FileImage(imagePath + ringParameters.imageParameters.file_name);
                RingReturn ringReturn = ringRecognition.findGoldRings(fileImage, ringParameters);
                if (ringReturn.fatalComputerVisionError)
                    throw new AutonomousRobotException(TAG, "Error in computer vision subsystem");

                RobotLogCommon.d(TAG, "Found Target Zone " + ringReturn.targetZone);
                displayResults(imagePath + ringParameters.imageParameters.file_name,
                        "Rings indicate " + ringReturn.targetZone,
                        "FTC 2020 - 2021 Ultimate Goal");

            break;
            }
            case "FIND_TEAM_SCORING_ELEMENT": {

                // Read the parameters for barcode recognition from the xml file.
                BarcodeParametersXML barcodeParametersXML = new BarcodeParametersXML(WorkingDirectory.getWorkingDirectory() + RobotConstants.xmlDir);
                BarcodeParameters barcodeParameters = barcodeParametersXML.getBarcodeParameters();
                BarcodeRecognition barcodeRecognition = new BarcodeRecognition();

                // Call the OpenCV subsystem.
                String imagePath = WorkingDirectory.getWorkingDirectory() + RobotConstants.imageDir;
                ImageProvider fileImage = new FileImage(imagePath + barcodeParameters.imageParameters.file_name);
                BarcodeReturn barcodeReturn = barcodeRecognition.findTeamScoringElement(fileImage, barcodeParameters);
                if (barcodeReturn.fatalComputerVisionError)
                    throw new AutonomousRobotException(TAG, "Error in computer vision subsystem");

                RobotLogCommon.d(TAG, "Found Team Scoring Element " + barcodeReturn.barcodePosition);
                displayResults(imagePath + barcodeParameters.imageParameters.file_name,
                        "Team Scoring Element at position " + barcodeReturn.barcodePosition,
                        "FTC 2021 - 2022 Freight Frenzy");

                break;
            }
            default:
                throw new AutonomousRobotException(TAG, "Unrecognized image recognition action");
        }

        RobotLogCommon.closeLog();
    }

    // Display the image in the Pane.
    private void displayResults(String pImageFile, String pResultText, String pTitle) throws FileNotFoundException {
        InputStream stream = new FileInputStream(pImageFile);
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
        Text ringText = new Text(pResultText);
        ringText.setFont(NOTIFICATION_TEXT_FONT);
        ringText.setFill(Color.CYAN);
        ringText.setX(NOTIFICATION_TEXT_POSITION_X);
        ringText.setY(NOTIFICATION_TEXT_POSITION_Y);
        field.getChildren().add(ringText);

        Scene scene = new Scene(field, FIELD_WIDTH, FIELD_HEIGHT, Color.GRAY);
        stage.setTitle(pTitle);
        stage.setScene(scene);
        stage.show();
    }

}