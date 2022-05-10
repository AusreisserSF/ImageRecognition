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
import org.firstinspires.ftc.ftcdevcommon.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.RobotXMLElement;
import org.firstinspires.ftc.ftcdevcommon.XPathAccess;
import org.firstinspires.ftc.ftcdevcommon.intellij.WorkingDirectory;
import org.firstinspires.ftc.teamcode.auto.vision.*;
import org.firstinspires.ftc.teamcode.auto.xml.*;
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

public class RecognitionDispatcher extends Application {

    private static final String TAG = RecognitionDispatcher.class.getSimpleName();

    public static final double FIELD_WIDTH = 410;
    public static final double FIELD_HEIGHT = 300;

    public static final double NOTIFICATION_TEXT_POSITION_Y = (FIELD_HEIGHT / 2) + 20;
    public static final double NOTIFICATION_TEXT_POSITION_X = 10;
    public static final Font NOTIFICATION_TEXT_FONT = Font.font("Comic Sans MS", FontWeight.BOLD, 16);

    private Stage stage;
    private Pane field;
    private String imagePath;
    private RobotActionXMLFreightFrenzy robotActionXMLFreightFrenzy;
    private RobotActionXMLGoldCube robotActionXMLGoldCube;

    // Load OpenCV.
    private static final boolean openCVInitialized;

    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME); // IntelliJ only
        openCVInitialized = true; // IntelliJ only
    }

    // See https://stackoverflow.com/questions/21729362/trying-to-call-a-javafx-application-from-java-nosuchmethodexception
    //!! Default constructor is required.
    public RecognitionDispatcher() {
        RobotLogCommon.initialize(RobotLogCommon.LogIdentifier.TEST_LOG, WorkingDirectory.getWorkingDirectory() + RobotConstants.logDir);
        RobotLogCommon.i(TAG, "Starting image recognition");

        if (!openCVInitialized)
            throw new AutonomousRobotException(TAG, "Failure in OpenCV initialization");
    }

    @Override
    public void start(Stage pStage) throws Exception {
        stage = pStage;
        field = new Pane();
        imagePath = WorkingDirectory.getWorkingDirectory() + RobotConstants.imageDir;

        // Process the command line parameters common.
        Parameters parameters = getParameters();
        Map<String, String> namedParameters = parameters.getNamed();

        String gameParameter = namedParameters.get("game");
        if (gameParameter == null)
            throw new AutonomousRobotException(TAG, "Required parameter --game is missing");

        RobotLogCommon.c(TAG, "Game " + gameParameter);

        // Get the name of the file that contains the robot's actions,
        // e.g. RobotAction.xml.
        String actionXMLFilenameParameter = namedParameters.get("xml");
        if (actionXMLFilenameParameter == null)
            throw new AutonomousRobotException(TAG, "Required parameter --xml is missing");

        // Use RobotAction.xml but for the single OpMode TEST with a single action only.
        List<RobotXMLElement> actions;
        switch (gameParameter) {
            case "TEST": {
                robotActionXMLGoldCube = new RobotActionXMLGoldCube(WorkingDirectory.getWorkingDirectory() + RobotConstants.xmlDir + actionXMLFilenameParameter);
                RobotActionXMLGoldCube.RobotActionDataGoldCube actionData = robotActionXMLGoldCube.getOpModeData("TEST");
                actions = actionData.actions;
                if (actions.size() != 1)
                    throw new AutonomousRobotException(TAG, "TEST OpMode must contain a single action");

                String recognitionAction = actions.get(0).getRobotXMLElementName();
                if (!((recognitionAction.equals("ANALYZE_BARCODE") || recognitionAction.equals("APPROACH_SHIPPING_HUB_BY_VISION"))))
                    throw new AutonomousRobotException(TAG, "Missing required action ANALYZE_BARCODE or APPROACH_SHIPPING_BY_WITH_VISION");
                break;
            }

            case "Freight Frenzy": {
               robotActionXMLFreightFrenzy = new RobotActionXMLFreightFrenzy(WorkingDirectory.getWorkingDirectory() + RobotConstants.xmlDir);
               RobotActionXMLFreightFrenzy.RobotActionDataFreightFrenzy actionData = robotActionXMLFreightFrenzy.getOpModeData("TEST");
               actions = actionData.actions;
               if (actions.size() != 1)
                    throw new AutonomousRobotException(TAG, "TEST OpMode must contain a single action");

               String recognitionAction = actions.get(0).getRobotXMLElementName();
               if (!((recognitionAction.equals("ANALYZE_BARCODE") || recognitionAction.equals("APPROACH_SHIPPING_HUB_BY_VISION"))))
                    throw new AutonomousRobotException(TAG, "Missing required action ANALYZE_BARCODE or APPROACH_SHIPPING_BY_WITH_VISION");
               break;
            }

            /*
            case "Ultimate Goal": {
                if (!recognitionAction.equals("RECOGNIZE_RINGS"))
                    throw new AutonomousRobotException(TAG, "Missing required action RECOGNIZE_RINGS");
                break;
            }
             */
            default:
                throw new AutonomousRobotException(TAG, "Unrecognized game " + gameParameter);
        }

        // Set up XPath access to the current action.
        RobotXMLElement actionElement = actions.get(0);
        XPathAccess actionXPath = new XPathAccess(actionElement);

        String actionName = actionElement.getRobotXMLElementName().toUpperCase();
        RobotLogCommon.d(TAG, "Executing action " + actionName);
        switch (actionName) {

            // Summer 2022: test Intel Realsense depth camera(s).
            case "GOLD_CUBE_DEPTH": {
                //**TODO
                break;
            }

             // Freight Frenzy 2021-2022
            case "ANALYZE_BARCODE": {
                // Read the parameters for barcode recognition from the xml file.
                BarcodeParametersXML barcodeParametersXML = new BarcodeParametersXML(WorkingDirectory.getWorkingDirectory() + RobotConstants.xmlDir);
                BarcodeParameters barcodeParameters = barcodeParametersXML.getBarcodeParameters();
                BarcodeRecognition barcodeRecognition = new BarcodeRecognition();

                // Get the <image_parameters> for the barcode from the XML file.
                VisionParameters.ImageParameters barcodeImageParameters =
                        robotActionXMLFreightFrenzy.getImageParametersFromXPath(actionElement, "image_parameters");

                // Make sure that this tester is reading the image from a file.
                if (!(barcodeImageParameters.ocv_image.endsWith(".png") ||
                        barcodeImageParameters.ocv_image.endsWith(".jpg")))
                    throw new AutonomousRobotException(TAG, "Invalid image file name");

                ImageProvider fileImage = new FileImage(imagePath + barcodeImageParameters.ocv_image);

                // Get the recognition path from the XML file.
                String recognitionPathString = actionXPath.getRequiredString("barcode_recognition/recognition_path");
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
                int left_x = actionXPath.getRequiredInt("barcode_recognition/left_window/x");
                int left_y = actionXPath.getRequiredInt("barcode_recognition/left_window/y");
                int left_width = actionXPath.getRequiredInt("barcode_recognition/left_window/width");
                int left_height = actionXPath.getRequiredInt("barcode_recognition/left_window/height");
                RobotConstantsFreightFrenzy.ShippingHubLevels left_shipping_hub_level = RobotConstantsFreightFrenzy.ShippingHubLevels.valueOf(actionXPath.getRequiredString("barcode_recognition/left_window/shipping_hub_level").toUpperCase());
                barcodeElements.put(RobotConstantsFreightFrenzy.BarcodeElementWindow.LEFT, new Rect(left_x, left_y, left_width, left_height));

                // Get the boundaries for the right window onto the barcode.
                int right_width = actionXPath.getRequiredInt("barcode_recognition/right_window/width");
                RobotConstantsFreightFrenzy.ShippingHubLevels right_shipping_hub_level = RobotConstantsFreightFrenzy.ShippingHubLevels.valueOf(actionXPath.getRequiredString("barcode_recognition/right_window/shipping_hub_level").toUpperCase());
                // Note: the right window starts 1 pixel past the left element. The height of the right
                // window is the same as that of the left window.
                barcodeElements.put(RobotConstantsFreightFrenzy.BarcodeElementWindow.RIGHT, new Rect(left_x + left_width + 1, left_y, right_width, left_height));
                barcodeParameters.setBarcodeElements(barcodeElements);

                // Set the shipping hub level to infer if the Shipping Hub Element is either the left or right window.
                RobotConstantsFreightFrenzy.ShippingHubLevels npos_shipping_hub_level = RobotConstantsFreightFrenzy.ShippingHubLevels.valueOf(actionXPath.getRequiredString("barcode_recognition/barcode_element_npos/shipping_hub_level").toUpperCase());

                // At last perform the image recognition.
                BarcodeReturn barcodeReturn =
                        barcodeRecognition.findTeamScoringElement(fileImage, barcodeImageParameters, barcodeParameters,
                                recognitionPath);
                if (barcodeReturn.openCVResults == RobotConstants.OpenCVResults.OCV_ERROR)
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
                displayResults(imagePath + barcodeImageParameters.ocv_image,
                        shippingHubLevel.toString(),
                        "FTC 2021 - 2022 Freight Frenzy");
                break;
            }

            // Use Dennis's method, which requires that we calculate the distance
            // using the same picture that we used for the angle, even if the Shipping
            // Hub is not at the center of the image.
            // Combine shippingHubRecognition.getAngleToShippingHub and
            // getDistanceToShippingHub into getAngleAndDistanceToShippingHub.

            //## 3/15/2022 See notes in getAngleAndDistanceToShippingHub.
            case "APPROACH_SHIPPING_HUB_BY_VISION": {
                // This action needs a command line switch of --alliance=["BLUE" | "RED"]
                String allianceString = namedParameters.get("alliance");
                RobotConstants.Alliance alliance = RobotConstants.Alliance.valueOf(allianceString);
                RobotLogCommon.c(TAG, "Alliance " + alliance);

                // Read the parameters for the Shipping Hub from the xml file.
                ShippingHubParametersXML shippingHubParametersXML = new ShippingHubParametersXML(WorkingDirectory.getWorkingDirectory() + RobotConstants.xmlDir);
                ShippingHubParameters shippingHubParameters = shippingHubParametersXML.getShippingHubParameters();
                ShippingHubRecognition shippingHubRecognition = new ShippingHubRecognition();

                // Get the <image_parameters> for the Shipping Hub from the XML file.
                VisionParameters.ImageParameters shippingHubImageParameters =
                        robotActionXMLFreightFrenzy.getImageParametersFromXPath(actionElement, "image_parameters");

                // Make sure that this tester is reading the image from a file.
                if (!(shippingHubImageParameters.ocv_image.endsWith(".png") ||
                        shippingHubImageParameters.ocv_image.endsWith(".jpg")))
                    throw new AutonomousRobotException(TAG, "Invalid image file name");

                ImageProvider fileImage = new FileImage(imagePath + shippingHubImageParameters.ocv_image);

                // Depending on the OpMode get the BLUE or RED HSV parameters.
                VisionParameters.HSVParameters shHSVParameters;
                if (alliance == RobotConstants.Alliance.BLUE)
                    shHSVParameters = shippingHubParameters.blueAllianceHSVParameters;
                else if (alliance == RobotConstants.Alliance.RED)
                    shHSVParameters = shippingHubParameters.redAllianceHSVParameters;
                else
                    throw new AutonomousRobotException(TAG, "Shipping Hub alliance must be BLUE or RED");

                // Get the angle and distance of the robot to the Shipping Hub.
                ShippingHubReturn shReturn = shippingHubRecognition.getAngleAndDistanceToShippingHub(fileImage, shippingHubImageParameters, shHSVParameters, shippingHubParameters);
                if (shReturn.openCVResults == RobotConstants.OpenCVResults.OCV_ERROR) {
                    RobotLogCommon.d(TAG, "Error in OpenCV processing during getAngleAndDistanceToShippingHub");
                    break;
                }

                if (shReturn.angleToShippingHub == ShippingHubReturn.SHIPPING_HUB_ANGLE_NPOS) {
                    RobotLogCommon.d(TAG, "Unable to compute angle to Shipping Hub");
                    break;
                }

                if (shReturn.distanceToShippingHub == ShippingHubReturn.SHIPPING_HUB_DISTANCE_NPOS) {
                    RobotLogCommon.d(TAG, "Unable to compute distance to Shipping Hub");
                    break;
                }

                String displayText = "Distance from camera to Shipping Hub " + String.format("%.2f", shReturn.distanceToShippingHub) +
                        '\n' + "Angle from robot center to Shipping Hub " + String.format("%.2f", shReturn.angleToShippingHub);

                displayResults(imagePath + shippingHubImageParameters.ocv_image,
                        displayText,
                        "FTC 2021 - 2022 Freight Frenzy");

                break;
            }

            // Ultimate Goal 2020-2021
            //## 2/27/2022 NOT reachable; keep for reference/technique
            case "RECOGNIZE_RINGS": {
                // Read the parameters for ring recognition from the xml file.
                RingParametersXML ringParametersXML = new RingParametersXML(WorkingDirectory.getWorkingDirectory() + RobotConstants.xmlDir);
                RingParameters ringParameters = ringParametersXML.getRingParameters();
                RingRecognition ringRecognition = new RingRecognition();

                // Call the OpenCV subsystem.
                ImageProvider fileImage = new FileImage(imagePath + ringParameters.imageParameters.ocv_image);
                RingReturn ringReturn = ringRecognition.findGoldRings(fileImage, ringParameters);
                if (ringReturn.fatalComputerVisionError)
                    throw new AutonomousRobotException(TAG, "Error in computer vision subsystem");

                RobotLogCommon.d(TAG, "Found Target Zone " + ringReturn.targetZone);
                displayResults(imagePath + ringParameters.imageParameters.ocv_image,
                        "Rings indicate " + ringReturn.targetZone,
                        "FTC 2020 - 2021 Ultimate Goal");
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
        Text displayText = new Text(pResultText);
        displayText.setFont(NOTIFICATION_TEXT_FONT);
        displayText.setFill(Color.CYAN);
        displayText.setX(NOTIFICATION_TEXT_POSITION_X);
        displayText.setY(NOTIFICATION_TEXT_POSITION_Y);
        field.getChildren().add(displayText);

        Scene scene = new Scene(field, FIELD_WIDTH, FIELD_HEIGHT, Color.GRAY);
        stage.setTitle(pTitle);
        stage.setScene(scene);
        stage.show();
    }

}