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
import org.firstinspires.ftc.teamcode.common.RobotConstantsPowerPlay;
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
    private RobotActionXMLFreightFrenzy robotActionXMLFreightFrenzy;
    private RobotActionXMLStandard robotActionXMLGoldCube;
    private RobotActionXMLStandard robotActionXMLSignalSleeve;
    private RobotActionXMLStandard robotActionXMLConeStack;
    private RobotActionXMLStandard robotActionXMLJunction;

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
        String imagePath = WorkingDirectory.getWorkingDirectory() + RobotConstants.imageDir;

        // Process the command line parameters common.
        Parameters parameters = getParameters();
        Map<String, String> namedParameters = parameters.getNamed();

        String opmodeParameter = namedParameters.get("opmode");
        if (opmodeParameter == null)
            throw new AutonomousRobotException(TAG, "Required parameter --opmode is missing");

        RobotLogCommon.c(TAG, "OpMode " + opmodeParameter);

        RobotConstants.Alliance alliance = RobotConstants.Alliance.NONE;
        String allianceParameter = namedParameters.get("alliance"); // optional
        if (allianceParameter != null)
            alliance = RobotConstants.Alliance.valueOf(allianceParameter);

        RobotLogCommon.c(TAG, "Alliance " + alliance);

        // Get the name of the file that contains the robot's actions,
        // e.g. RobotAction.xml.
        String actionXMLFilenameParameter = namedParameters.get("xml");
        if (actionXMLFilenameParameter == null)
            throw new AutonomousRobotException(TAG, "Required parameter --xml is missing");

        // Use variations of RobotAction.xml (see the command line) with a single OpMode
        // that has a single action.
        List<RobotXMLElement> actions;
        switch (opmodeParameter) {
            case "TEST" -> {
                   throw new AutonomousRobotException(TAG, "TEST OpMode not implemented");
            }

            case "SIGNAL_SLEEVE" -> {
                robotActionXMLSignalSleeve = new RobotActionXMLStandard(WorkingDirectory.getWorkingDirectory() + RobotConstants.xmlDir + actionXMLFilenameParameter);
                RobotActionXMLStandard.RobotActionDataStandard actionData = robotActionXMLSignalSleeve.getOpModeData("SIGNAL_SLEEVE");
                actions = actionData.actions;
                if (actions.size() != 1)
                    throw new AutonomousRobotException(TAG, "SIGNAL_SLEEVE OpMode must contain a single action");

                String recognitionAction = actions.get(0).getRobotXMLElementName();
                if (!recognitionAction.equals("ANALYZE_SIGNAL_SLEEVE"))
                    throw new AutonomousRobotException(TAG, "Missing required action ANALYZE_SIGNAL_SLEEVE");
            }

            case "CONE_STACK" -> {
                robotActionXMLConeStack = new RobotActionXMLStandard(WorkingDirectory.getWorkingDirectory() + RobotConstants.xmlDir + actionXMLFilenameParameter);
                RobotActionXMLStandard.RobotActionDataStandard actionData = robotActionXMLConeStack.getOpModeData("CONE_STACK");
                actions = actionData.actions;
                if (actions.size() != 1)
                    throw new AutonomousRobotException(TAG, "CONE_STACK OpMode must contain a single action");

                String recognitionAction = actions.get(0).getRobotXMLElementName();
                if (!recognitionAction.equals("CONE_STACK_DEPTH"))
                    throw new AutonomousRobotException(TAG, "Missing required action CONE_STACK_DEPTH");
            }

            case "JUNCTION" -> {
                robotActionXMLJunction = new RobotActionXMLStandard(WorkingDirectory.getWorkingDirectory() + RobotConstants.xmlDir + actionXMLFilenameParameter);
                RobotActionXMLStandard.RobotActionDataStandard actionData = robotActionXMLJunction.getOpModeData("JUNCTION");
                actions = actionData.actions;
                if (actions.size() != 1)
                    throw new AutonomousRobotException(TAG, "JUNCTION OpMode must contain a single action");

                String recognitionAction = actions.get(0).getRobotXMLElementName();
                if (!recognitionAction.equals("JUNCTION_DEPTH"))
                    throw new AutonomousRobotException(TAG, "Missing required action JUNCTION_DEPTH");
            }

            case "GOLD_CUBE" -> {
                robotActionXMLGoldCube = new RobotActionXMLStandard(WorkingDirectory.getWorkingDirectory() + RobotConstants.xmlDir + actionXMLFilenameParameter);
                RobotActionXMLStandard.RobotActionDataStandard actionData = robotActionXMLGoldCube.getOpModeData("GOLD_CUBE");
                actions = actionData.actions;
                if (actions.size() != 1)
                    throw new AutonomousRobotException(TAG, "GOLD_CUBE OpMode must contain a single action");

                String recognitionAction = actions.get(0).getRobotXMLElementName();
                if (!recognitionAction.equals("GOLD_CUBE_DEPTH"))
                    throw new AutonomousRobotException(TAG, "Missing required action GOLD_CUBE_DEPTH");
            }

            case "Freight Frenzy" -> {
                robotActionXMLFreightFrenzy = new RobotActionXMLFreightFrenzy(WorkingDirectory.getWorkingDirectory() + RobotConstants.xmlDir);
                RobotActionXMLFreightFrenzy.RobotActionDataFreightFrenzy actionData = robotActionXMLFreightFrenzy.getOpModeData("TEST");
                actions = actionData.actions;
                if (actions.size() != 1)
                    throw new AutonomousRobotException(TAG, "TEST OpMode must contain a single action");

                String recognitionAction = actions.get(0).getRobotXMLElementName();
                if (!((recognitionAction.equals("ANALYZE_BARCODE") || recognitionAction.equals("APPROACH_SHIPPING_HUB_BY_VISION"))))
                    throw new AutonomousRobotException(TAG, "Missing required action ANALYZE_BARCODE or APPROACH_SHIPPING_BY_WITH_VISION");
            }

            /*
            case "Ultimate Goal": {
                if (!recognitionAction.equals("RECOGNIZE_RINGS"))
                    throw new AutonomousRobotException(TAG, "Missing required action RECOGNIZE_RINGS");
                break;
            }
             */
            default -> throw new AutonomousRobotException(TAG, "Unrecognized game " + opmodeParameter);
        }

        // Set up XPath access to the current action.
        RobotXMLElement actionElement = actions.get(0);
        XPathAccess actionXPath = new XPathAccess(actionElement);

        //**TODO all OpModes - if return == NPOS display "unable to determine"
        String actionName = actionElement.getRobotXMLElementName().toUpperCase();
        RobotLogCommon.d(TAG, "Executing action " + actionName);
        String imageFilename;
        switch (actionName) {
            case "ANALYZE_SIGNAL_SLEEVE" -> {
                // Read the parameters for signal sleeve recognition from the xml file.
                SignalSleeveParametersXML signalSleeveParametersXML = new SignalSleeveParametersXML(WorkingDirectory.getWorkingDirectory() + RobotConstants.xmlDir);
                SignalSleeveParameters signalSleeveParameters = signalSleeveParametersXML.getSignalSleeveParameters();

                // Get the <image_parameters> for the signal sleeve from the RobotAction (+suffix) XML file.
                VisionParameters.ImageParameters signalSleeveImageParameters =
                        robotActionXMLSignalSleeve.getImageParametersFromXPath(actionElement, "image_parameters");

                // Make sure that this tester is reading the image from a file.
                if (!(signalSleeveImageParameters.image_source.endsWith(".png") ||
                        signalSleeveImageParameters.image_source.endsWith(".jpg")))
                    throw new AutonomousRobotException(TAG, "Invalid image file name");

                imageFilename = signalSleeveImageParameters.image_source;
                ImageProvider fileImage = new FileImage(imagePath + imageFilename);

                // Get the recognition path from the XML file.
                String recognitionPathString = actionXPath.getRequiredString("signal_sleeve_recognition/recognition_path");
                RobotConstantsPowerPlay.SignalSleeveRecognitionPath signalSleeveRecognitionPath =
                        RobotConstantsPowerPlay.SignalSleeveRecognitionPath.valueOf(recognitionPathString.toUpperCase());

                RobotLogCommon.d(TAG, "Recognition path " + signalSleeveRecognitionPath);

                // Perform image recognition and depth mapping.
                SignalSleeveRecognition recognition = new SignalSleeveRecognition();
                SignalSleeveReturn signalSleeveReturn = recognition.recognizeSignalSleeve(fileImage, signalSleeveImageParameters, signalSleeveParameters, alliance, signalSleeveRecognitionPath);
                String displayText = "Image: " + imageFilename +
                        '\n' + "Signal sleeve location: " + signalSleeveReturn.signalSleeveLocation;

                displayResults(imagePath + signalSleeveImageParameters.image_source,
                        displayText,
                        "Test signal sleeve recognition");
            }

            case "CONE_STACK_DEPTH" -> {
                String xmlDir = WorkingDirectory.getWorkingDirectory() + RobotConstants.xmlDir;

                // Read the parameters for cone stack recognition from the xml file.
                ConeStackParametersXML coneStackParametersXML = new ConeStackParametersXML(xmlDir);
                ConeStackParameters coneStackParameters = coneStackParametersXML.getConeStackParameters();

                D405ConfigurationXML d405ConfigurationXML = new D405ConfigurationXML(xmlDir);
                D405Configuration d405Configuration = d405ConfigurationXML.getD405Configuration();

                // Get the <image_parameters> for the signal sleeve from the RobotAction (+suffix) XML file.
                VisionParameters.ImageParameters coneStackImageParameters =
                        robotActionXMLConeStack.getImageParametersFromXPath(actionElement, "image_parameters");

                // Make sure that this tester is reading the image from a file.
                if (!(coneStackImageParameters.image_source.endsWith(".png") ||
                        coneStackImageParameters.image_source.endsWith(".jpg")))
                    throw new AutonomousRobotException(TAG, "Invalid image file name");

                imageFilename = coneStackImageParameters.image_source;
                ImageProvider fileImage = new FileImage(imagePath + imageFilename);

                // Get the recognition path from the XML file.
                String recognitionPathString = actionXPath.getRequiredString("cone_stack_recognition/recognition_path");
                RobotConstantsPowerPlay.ConeStackRecognitionPath coneStackRecognitionPath =
                        RobotConstantsPowerPlay.ConeStackRecognitionPath.valueOf(recognitionPathString.toUpperCase());

                RobotLogCommon.d(TAG, "Recognition path " + coneStackRecognitionPath);

                // Perform image recognition and depth mapping.
                ConeStackRecognition coneStackRecognition = new ConeStackRecognition(alliance);
                RealSenseReturn realSenseReturn =
                        coneStackRecognition.recognizeConeStack(fileImage, d405Configuration, RobotConstantsPowerPlay.D405CameraId.SWIVEL, coneStackImageParameters, coneStackParameters, coneStackRecognitionPath);

                String distanceString = realSenseReturn.distanceFromRobotCenter == RealSenseReturn.RECOGNITION_DISTANCE_NPOS ? "unable to determine" : String.format("%.2f", realSenseReturn.distanceFromRobotCenter);
                String angleString = realSenseReturn.angleFromRobotCenter == RealSenseReturn.RECOGNITION_ANGLE_NPOS ? "unable to determine" : String.format("%.2f", realSenseReturn.angleFromRobotCenter);
                String displayText = "Image: " + imageFilename +
                        '\n' + "Center of robot to pixel on cone:" +
                        '\n' + "  Distance: " + distanceString + " inches" +
                        '\n' + "  Angle " + angleString + " degrees";

                displayResults(imagePath + coneStackImageParameters.image_source,
                        displayText,
                        "Test cone stack recognition");
            }

            case "JUNCTION_DEPTH" -> {
                String xmlDir = WorkingDirectory.getWorkingDirectory() + RobotConstants.xmlDir;

                // Read the parameters for junction recognition from the xml file.
                JunctionParametersXML junctionParametersXML = new JunctionParametersXML(xmlDir);
                JunctionParameters junctionParameters = junctionParametersXML.getJunctionParameters();

                D405ConfigurationXML d405ConfigurationXML = new D405ConfigurationXML(xmlDir);
                D405Configuration d405Configuration = d405ConfigurationXML.getD405Configuration();

                // Get the <image_parameters> for the object from the RobotAction (+suffix) XML file.
                VisionParameters.ImageParameters junctionImageParameters =
                        robotActionXMLJunction.getImageParametersFromXPath(actionElement, "image_parameters");

                // Make sure that this tester is reading the image from a file.
                if (!(junctionImageParameters.image_source.endsWith(".png") ||
                        junctionImageParameters.image_source.endsWith(".jpg")))
                    throw new AutonomousRobotException(TAG, "Invalid image file name");

                imageFilename = junctionImageParameters.image_source;
                ImageProvider fileImage = new FileImage(imagePath + junctionImageParameters.image_source);

                // Perform image recognition and depth mapping.
                // Get the recognition path from the XML file.
                String recognitionPathString = actionXPath.getRequiredString("junction_recognition/recognition_path");
                RobotConstantsPowerPlay.JunctionRecognitionPath junctionRecognitionPath =
                        RobotConstantsPowerPlay.JunctionRecognitionPath.valueOf(recognitionPathString.toUpperCase());

                RobotLogCommon.d(TAG, "Recognition path " + junctionRecognitionPath);

                // Perform image recognition and depth mapping.
                JunctionRecognition junctionRecognition = new JunctionRecognition(alliance);
                RealSenseReturn realSenseReturn =
                        junctionRecognition.recognizeJunction(fileImage, d405Configuration, RobotConstantsPowerPlay.D405CameraId.SWIVEL, junctionImageParameters, junctionParameters, junctionRecognitionPath);

                String distanceString = realSenseReturn.distanceFromRobotCenter == RealSenseReturn.RECOGNITION_DISTANCE_NPOS ? "unable to determine" : String.format("%.2f", realSenseReturn.distanceFromRobotCenter);
                String angleString = realSenseReturn.angleFromRobotCenter == RealSenseReturn.RECOGNITION_ANGLE_NPOS ? "unable to determine" : String.format("%.2f", realSenseReturn.angleFromRobotCenter);
                String displayText = "Image: " + imageFilename +
                        '\n' + "Center of robot to pixel on junction:" +
                        '\n' + "  Distance: " + distanceString + " inches" +
                        '\n' + "  Angle " + angleString + " degrees";

                displayResults(imagePath + junctionImageParameters.image_source,
                        displayText,
                        "Test junction recognition");
            }

            // Summer 2022: test Intel Realsense depth camera(s).
            case "GOLD_CUBE_DEPTH" -> {
                String xmlDir = WorkingDirectory.getWorkingDirectory() + RobotConstants.xmlDir;

                // Read the parameters for gold cube recognition from the xml file.
                GoldCubeParametersXML goldCubeParametersXML = new GoldCubeParametersXML(WorkingDirectory.getWorkingDirectory() + RobotConstants.xmlDir);
                GoldCubeParameters goldCubeParameters = goldCubeParametersXML.getGoldCubeParameters();

                D405ConfigurationXML d405ConfigurationXML = new D405ConfigurationXML(xmlDir);
                D405Configuration d405Configuration = d405ConfigurationXML.getD405Configuration();

                // Get the <image_parameters> for the gold cube from the RobotAction (+suffix) XML file.
                VisionParameters.ImageParameters goldCubeImageParameters =
                        robotActionXMLGoldCube.getImageParametersFromXPath(actionElement, "image_parameters");

                // Make sure that this tester is reading the image from a file.
                if (!(goldCubeImageParameters.image_source.endsWith(".png") ||
                        goldCubeImageParameters.image_source.endsWith(".jpg")))
                    throw new AutonomousRobotException(TAG, "Invalid image file name");

                imageFilename = goldCubeImageParameters.image_source;
                ImageProvider fileImage = new FileImage(imagePath + goldCubeImageParameters.image_source);

                // Perform image recognition and depth mapping.
                // Get the recognition path from the XML file.
                String recognitionPathString = actionXPath.getRequiredString("gold_cube_recognition/recognition_path");
                RobotConstants.RecognitionPath goldCubeRecognitionPath =
                        RobotConstants.RecognitionPath.valueOf(recognitionPathString.toUpperCase());

                RobotLogCommon.d(TAG, "Recognition path " + goldCubeRecognitionPath);

                // Perform image recognition and depth mapping.
                GoldCubeRecognition goldCubeRecognition = new GoldCubeRecognition(alliance);
                RealSenseReturn realSenseReturn =
                        goldCubeRecognition.recognizeGoldCube(fileImage, d405Configuration, RobotConstantsPowerPlay.D405CameraId.SWIVEL, goldCubeImageParameters, goldCubeParameters, goldCubeRecognitionPath);

                String distanceString = realSenseReturn.distanceFromRobotCenter == RealSenseReturn.RECOGNITION_DISTANCE_NPOS ? "unable to determine" : String.format("%.2f", realSenseReturn.distanceFromRobotCenter);
                String angleString = realSenseReturn.angleFromRobotCenter == RealSenseReturn.RECOGNITION_ANGLE_NPOS ? "unable to determine" : String.format("%.2f", realSenseReturn.angleFromRobotCenter);
                String displayText = "Image: " + imageFilename +
                        '\n' + "Center of robot to pixel on gold cube:" +
                        '\n' + "  Distance: " + distanceString + " inches" +
                        '\n' + "  Angle " + angleString + " degrees";

                displayResults(imagePath + goldCubeImageParameters.image_source,
                        displayText,
                        "Test gold cube recognition");
            }

            // Freight Frenzy 2021-2022
            case "ANALYZE_BARCODE" -> {
                // Read the parameters for barcode recognition from the xml file.
                BarcodeParametersXML barcodeParametersXML = new BarcodeParametersXML(WorkingDirectory.getWorkingDirectory() + RobotConstants.xmlDir);
                BarcodeParameters barcodeParameters = barcodeParametersXML.getBarcodeParameters();
                BarcodeRecognition barcodeRecognition = new BarcodeRecognition();

                // Get the <image_parameters> for the barcode from the XML file.
                VisionParameters.ImageParameters barcodeImageParameters =
                        robotActionXMLFreightFrenzy.getImageParametersFromXPath(actionElement, "image_parameters");

                // Make sure that this tester is reading the image from a file.
                if (!(barcodeImageParameters.image_source.endsWith(".png") ||
                        barcodeImageParameters.image_source.endsWith(".jpg")))
                    throw new AutonomousRobotException(TAG, "Invalid image file name");

                ImageProvider fileImage = new FileImage(imagePath + barcodeImageParameters.image_source);

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
                    case LEFT -> shippingHubLevel = left_shipping_hub_level;
                    case RIGHT -> shippingHubLevel = right_shipping_hub_level;
                    case WINDOW_NPOS -> shippingHubLevel = npos_shipping_hub_level;
                    default -> throw new AutonomousRobotException(TAG, "Unrecognized enum value " + barcodeReturn.barcodeElementWindow);
                }

                RobotLogCommon.d(TAG, "Found Team Scoring Element at position " + barcodeReturn.barcodeElementWindow);
                RobotLogCommon.d(TAG, "Shipping Hub Level " + shippingHubLevel);
                displayResults(imagePath + barcodeImageParameters.image_source,
                        shippingHubLevel.toString(),
                        "FTC 2021 - 2022 Freight Frenzy");
            }


            // Use Dennis's method, which requires that we calculate the distance
            // using the same picture that we used for the angle, even if the Shipping
            // Hub is not at the center of the image.
            // Combine shippingHubRecognition.getAngleToShippingHub and
            // getDistanceToShippingHub into getAngleAndDistanceToShippingHub.

            //## 3/15/2022 See notes in getAngleAndDistanceToShippingHub.
            case "APPROACH_SHIPPING_HUB_BY_VISION" -> {
                // Read the parameters for the Shipping Hub from the xml file.
                ShippingHubParametersXML shippingHubParametersXML = new ShippingHubParametersXML(WorkingDirectory.getWorkingDirectory() + RobotConstants.xmlDir);
                ShippingHubParameters shippingHubParameters = shippingHubParametersXML.getShippingHubParameters();
                ShippingHubRecognition shippingHubRecognition = new ShippingHubRecognition();

                // Get the <image_parameters> for the Shipping Hub from the XML file.
                VisionParameters.ImageParameters shippingHubImageParameters =
                        robotActionXMLFreightFrenzy.getImageParametersFromXPath(actionElement, "image_parameters");

                // Make sure that this tester is reading the image from a file.
                if (!(shippingHubImageParameters.image_source.endsWith(".png") ||
                        shippingHubImageParameters.image_source.endsWith(".jpg")))
                    throw new AutonomousRobotException(TAG, "Invalid image file name");

                ImageProvider fileImage = new FileImage(imagePath + shippingHubImageParameters.image_source);

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

                displayResults(imagePath + shippingHubImageParameters.image_source,
                        displayText,
                        "FTC 2021 - 2022 Freight Frenzy");
            }

            // Ultimate Goal 2020-2021
            //## 2/27/2022 NOT reachable; keep for reference/technique
            case "RECOGNIZE_RINGS" -> {
                // Read the parameters for ring recognition from the xml file.
                RingParametersXML ringParametersXML = new RingParametersXML(WorkingDirectory.getWorkingDirectory() + RobotConstants.xmlDir);
                RingParameters ringParameters = ringParametersXML.getRingParameters();
                RingRecognition ringRecognition = new RingRecognition();

                // Call the OpenCV subsystem.
                ImageProvider fileImage = new FileImage(imagePath + ringParameters.imageParameters.image_source);
                RingReturn ringReturn = ringRecognition.findGoldRings(fileImage, ringParameters);
                if (ringReturn.fatalComputerVisionError)
                    throw new AutonomousRobotException(TAG, "Error in computer vision subsystem");

                RobotLogCommon.d(TAG, "Found Target Zone " + ringReturn.targetZone);
                displayResults(imagePath + ringParameters.imageParameters.image_source,
                        "Rings indicate " + ringReturn.targetZone,
                        "FTC 2020 - 2021 Ultimate Goal");
            }
            default -> throw new AutonomousRobotException(TAG, "Unrecognized image recognition action");
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