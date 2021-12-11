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
import org.firstinspires.ftc.ftcdevcommon.*;
import org.firstinspires.ftc.ftcdevcommon.intellij.WorkingDirectory;
import org.firstinspires.ftc.teamcode.auto.vision.*;
import org.firstinspires.ftc.teamcode.auto.xml.BarcodeParametersXML;
import org.firstinspires.ftc.teamcode.auto.xml.RingParametersXML;
import org.firstinspires.ftc.teamcode.auto.xml.RobotActionXMLFreightFrenzy;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsFreightFrenzy;
import org.opencv.core.Core;
import org.opencv.core.Rect;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CompletableFuture;

public class RecognitionDispatcher extends Application {

    private static final String TAG = "RecognitionDispatcher";

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
        RobotLogCommon.i(TAG, "Starting barcode recognition");

        if (!openCVInitialized)
            throw new AutonomousRobotException(TAG, "Failure in OpenCV initialization");
     }

    @Override
    public void start(Stage pStage) throws Exception {
        stage = pStage;
        field = new Pane();

        // Process the command line parameters common.
        Parameters parameters = getParameters();
        Map<String, String> namedParameters = parameters.getNamed();

        String gameParameter = namedParameters.get("game");
        if (gameParameter == null)
            throw new AutonomousRobotException(TAG, "Required parameter --game is missing");

        if (!(gameParameter.equals("Freight Frenzy") || gameParameter.equals("Ultimate Goal")))
            throw new AutonomousRobotException(TAG, "Unrecognized game " + gameParameter);

        RobotLogCommon.c(TAG, "Game " + gameParameter);

        // Use RobotAction.xml but for the single OpMode TEST with a single action only.
        RobotActionXMLFreightFrenzy robotActionXMLFreightFrenzy = new RobotActionXMLFreightFrenzy(WorkingDirectory.getWorkingDirectory() + RobotConstants.xmlDir);
        RobotActionXMLFreightFrenzy.RobotActionDataFreightFrenzy actionData = robotActionXMLFreightFrenzy.getOpModeData("TEST");
        List<RobotXMLElement> actions = actionData.actions;
        if (actions.size() != 1)
            throw new AutonomousRobotException(TAG, "TEST OpMode must contain a single action");

        String recognitionAction = actions.get(0).getRobotXMLElementName();
        switch (gameParameter) {
            case "Freight Frenzy": {
                if (!recognitionAction.equals("ANALYZE_BARCODE"))
                    throw new AutonomousRobotException(TAG, "Missing required action ANALYZE_BARCODE");
                break;
            }
            case "Ultimate Goal": {
                if (!recognitionAction.equals("RECOGNIZE_RINGS"))
                    throw new AutonomousRobotException(TAG, "Missing required action RECOGNIZE_RINGS");
                break;
            }
            default:
                throw new AutonomousRobotException(TAG, "Unrecognized game " + gameParameter);
        }

        // Set up XPath access to the current action command.
        RobotXMLElement actionElement = actions.get(0);
        XPathAccess commandXPath = new XPathAccess(actionElement);

        // The only allowed image_provider is "file".
        String imageProviderId = commandXPath.getRequiredStringInRange("ocv_image_provider", commandXPath.validRange("vuforia", "file"));
        if (!imageProviderId.equals("file"))
            throw new AutonomousRobotException(TAG, "image_provider must be 'file'");

        String commandName = actionElement.getRobotXMLElementName().toUpperCase();
        switch (commandName) {

            // Ultimate Goal 2020-2021
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

            // Freight Frenzy 2021-2022
            case "ANALYZE_BARCODE": {
                // Read the parameters for barcode recognition from the xml file.
                BarcodeParametersXML barcodeParametersXML = new BarcodeParametersXML(WorkingDirectory.getWorkingDirectory() + RobotConstants.xmlDir);
                BarcodeParameters barcodeParameters = barcodeParametersXML.getBarcodeParameters();
                BarcodeRecognition barcodeRecognition = new BarcodeRecognition();

                // Prepare for image recognition.
                if (actionData.imageParameters == null)
                    throw new AutonomousRobotException(TAG, "Missing image_parameters");

                String imagePath = WorkingDirectory.getWorkingDirectory() + RobotConstants.imageDir;
                ImageProvider fileImage = new FileImage(imagePath + actionData.imageParameters.file_name);

                // Get the recognition path from the XML file.
                String recognitionPathString = commandXPath.getRequiredString("barcode_recognition/recognition_path");
                RobotConstantsFreightFrenzy.RecognitionPath recognitionPath;
                try {
                    recognitionPath = RobotConstantsFreightFrenzy.RecognitionPath.valueOf(recognitionPathString.toUpperCase());
                } catch (IllegalArgumentException iex) {
                    throw new AutonomousRobotException(TAG, "Invalid recognition path");
                }

                RobotLogCommon.c(TAG, "Recognition path " + recognitionPathString);

                // Set the barcode recognition parameters for the current OpMode.
                EnumMap<RobotConstantsFreightFrenzy.BarcodeElementWindow, Rect> barcodeElements =
                        new EnumMap<>(RobotConstantsFreightFrenzy.BarcodeElementWindow.class);

                // The left window onto the barcode may be the leftmost barcode element or the
                // center barcode element. The right window is always immediately to the right
                // of the left window.
                // Get the boundaries for the left window onto the barcode.
                int left_x = commandXPath.getRequiredInt("barcode_recognition/left_window/x");
                int left_y = commandXPath.getRequiredInt("barcode_recognition/left_window/y");
                int left_width = commandXPath.getRequiredInt("barcode_recognition/left_window/width");
                int left_height = commandXPath.getRequiredInt("barcode_recognition/left_window/height");
                RobotConstantsFreightFrenzy.ShippingHubLevels left_shipping_hub_level = RobotConstantsFreightFrenzy.ShippingHubLevels.valueOf(commandXPath.getRequiredString("barcode_recognition/left_window/shipping_hub_level").toUpperCase());
                barcodeElements.put(RobotConstantsFreightFrenzy.BarcodeElementWindow.LEFT, new Rect(left_x, left_y, left_width, left_height));

                // Get the boundaries for the right window onto the barcode.
                int right_width = commandXPath.getRequiredInt("barcode_recognition/right_window/width");
                RobotConstantsFreightFrenzy.ShippingHubLevels right_shipping_hub_level = RobotConstantsFreightFrenzy.ShippingHubLevels.valueOf(commandXPath.getRequiredString("barcode_recognition/right_window/shipping_hub_level").toUpperCase());
                // Note: the right window starts 1 pixel past the left element. The height of the right
                // window is the same as that of the left window.
                barcodeElements.put(RobotConstantsFreightFrenzy.BarcodeElementWindow.RIGHT, new Rect(left_x + left_width + 1, left_y, right_width, left_height));
                barcodeParameters.setBarcodeElements(barcodeElements);

                // Set the shipping hub level to infer if the Shipping Hub Element is either the left or right window.
                RobotConstantsFreightFrenzy.ShippingHubLevels npos_shipping_hub_level = RobotConstantsFreightFrenzy.ShippingHubLevels.valueOf(commandXPath.getRequiredString("barcode_recognition/barcode_element_npos/shipping_hub_level").toUpperCase());

                // At last perform the image recognition.
                BarcodeReturn barcodeReturn =
                        barcodeRecognition.findTeamScoringElement(fileImage, actionData.imageParameters, barcodeParameters,
                                recognitionPath);
                if (barcodeReturn.fatalComputerVisionError)
                    throw new AutonomousRobotException(TAG, "Error in computer vision subsystem");

                // Set the shipping hub level based on the barcode recognition.
                RobotConstantsFreightFrenzy.ShippingHubLevels shippingHubLevel;
                switch (barcodeReturn.barcodeElementWindow) {
                    case LEFT: {
                        shippingHubLevel = left_shipping_hub_level;
                        break;
                    }
                    case RIGHT: {
                        shippingHubLevel = right_shipping_hub_level;
                        break;
                    }
                    case WINDOW_NPOS: {
                        shippingHubLevel = npos_shipping_hub_level;
                        break;
                    }
                    default:
                        throw new AutonomousRobotException(TAG, "Unrecognized enum value " + barcodeReturn.barcodeElementWindow);
                }

                RobotLogCommon.d(TAG, "Found Team Scoring Element at position " + barcodeReturn.barcodeElementWindow);
                RobotLogCommon.d(TAG, "Shipping Hub Level " + shippingHubLevel);
                displayResults(imagePath + actionData.imageParameters.file_name,
                        shippingHubLevel.toString(),
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