package org.firstinspires.ftc.teamcode.auto.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.RobotXMLElement;
import org.firstinspires.ftc.ftcdevcommon.intellij.RobotLogCommon;
import org.firstinspires.ftc.teamcode.auto.vision.VisionParameters;
import org.firstinspires.ftc.teamcode.common.RobotConstantsFreightFrenzy;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPath;
import javax.xml.xpath.XPathConstants;
import javax.xml.xpath.XPathExpressionException;
import javax.xml.xpath.XPathFactory;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;

//**TODO Should supercede RobotActionXMLGoldCube
//**TODO For parsing RobotAction.xml in the standard format, i.e.
// without any insertions (see the RobotActionXMLFreightFrenzy).
//**TODO make others inherit from this class.
public class RobotActionXMLStandard {

    public static final String TAG = RobotActionXMLStandard.class.getSimpleName();

    private final String robotActionXMLFilename;
    private final Document document;
    private final XPath xpath;

    /*
    // IntelliJ only
    private static final String JAXP_SCHEMA_LANGUAGE =
            "http://java.sun.com/xml/jaxp/properties/schemaLanguage";
    private static final String W3C_XML_SCHEMA =
            "http://www.w3.org/2001/XMLSchema";
     */

    public RobotActionXMLStandard(String pActionXMLFilename) throws ParserConfigurationException, SAXException, IOException {

    /*
    // IntelliJ only
        DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
        dbFactory.setIgnoringComments(true);
        dbFactory.setNamespaceAware(true);
        dbFactory.setValidating(true);
        dbFactory.setAttribute(JAXP_SCHEMA_LANGUAGE, W3C_XML_SCHEMA);

        //## ONLY works with a validating parser (DTD or schema),
        // which the IntelliJ parser is.
        dbFactory.setIgnoringElementContentWhitespace(true);
    // End IntelliJ only
    */

    // Android or IntelliJ
        DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
        dbFactory.setIgnoringComments(true);
        //## ONLY works with a validating parser (DTD or schema),
        // which the Android Studio parser is not.
        // dbFactory.setIgnoringElementContentWhitespace(true);
        //PY 8/17/2019 Android throws UnsupportedOperationException dbFactory.setXIncludeAware(true);
    // End Android or IntelliJ

        DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
        robotActionXMLFilename = pActionXMLFilename;
        document = dBuilder.parse(new File(robotActionXMLFilename));

        XPathFactory xpathFactory = XPathFactory.newInstance();
        xpath = xpathFactory.newXPath();
    }

    //**TODO parsing of the <parameters> element is common. Put into ftcdevcommon
    // in a new package: xmlutils.
    // Reevaluate starting position and vumarks.

    // Find the requested opMode in the RobotActionGoldCube.xml file.
    // Package and return all data associated with the OpMode.
    public RobotActionDataStandard getOpModeData(String pOpMode) throws XPathExpressionException {

        Level logLevel = null; // null means use the default lowest logging level
        StartingPositionData startingPositionData = null;
        List<RobotConstantsFreightFrenzy.SupportedVumark> vumarksOfInterest = new ArrayList<>();
        List<RobotXMLElement> actions = new ArrayList<>();

        // Use XPath to locate the desired OpMode.
        String opModePath = "/RobotAction/OpMode[@id=" + "'" + pOpMode + "']";
        Node opModeNode = (Node) xpath.evaluate(opModePath, document, XPathConstants.NODE);
        if (opModeNode == null)
            throw new AutonomousRobotException(TAG, "Missing OpMode " + pOpMode);

        RobotLogCommon.c(TAG, "Extracting data from " + robotActionXMLFilename + " for OpMode " + pOpMode);

        // The next element in the XML is required: <parameters>
        Node parametersNode = getNextElement(opModeNode.getFirstChild());
        if ((parametersNode == null) || !parametersNode.getNodeName().equals("parameters"))
            throw new AutonomousRobotException(TAG, "Missing required <parameters> element");

        // The four possible elements under <parameters> are:
        //   <log_level>
        //   <starting_position>
        //   <vumarks>
        // All are optional.

        // A missing or empty optional logging_level will eventually return null, which
        // means to use the logger's default.
        Node nextParameterNode = getNextElement(parametersNode.getFirstChild());
        if ((nextParameterNode != null) && (nextParameterNode.getNodeName().equals("log_level"))) {
            String logLevelString = nextParameterNode.getTextContent().trim();
            if (!logLevelString.isEmpty()) {
                switch (logLevelString) {
                    case "d": {
                        logLevel = Level.FINE;
                        break;
                    }
                    case "v": {
                        logLevel = Level.FINER;
                        break;
                    }
                    case "vv": {
                        logLevel = Level.FINEST;
                        break;
                    }
                    default: {
                        throw new AutonomousRobotException(TAG, "Invalid logging level");
                    }
                }
            }
            nextParameterNode = getNextElement(nextParameterNode.getNextSibling());
        }

        // The next optional element in the XML is <starting_position>.
        if ((nextParameterNode != null) && nextParameterNode.getNodeName().equals("starting_position")) {
            // Get the value from each child of the starting_position element:
            // <x>79.0</x>
            // <y>188.0</y>
            // <angle>0.0</angle>
            double x;
            double y;
            double angle;
            Node xNode = getNextElement(nextParameterNode.getFirstChild());
            if ((xNode == null) || !xNode.getNodeName().equals("x") || xNode.getTextContent().isEmpty())
                throw new AutonomousRobotException(TAG, "Element 'x' missing or empty");

            try {
                x = Double.parseDouble(xNode.getTextContent());
            } catch (NumberFormatException nex) {
                throw new AutonomousRobotException(TAG, "Invalid number format in element 'x'");
            }

            Node yNode = getNextElement(xNode.getNextSibling());
            if ((yNode == null) || !yNode.getNodeName().equals("y") || yNode.getTextContent().isEmpty())
                throw new AutonomousRobotException(TAG, "Element 'y' missing or empty");

            try {
                y = Double.parseDouble(yNode.getTextContent());
            } catch (NumberFormatException nex) {
                throw new AutonomousRobotException(TAG, "Invalid number format in element 'y'");
            }

            Node angleNode = getNextElement(yNode.getNextSibling());
            if ((angleNode == null) || !angleNode.getNodeName().equals("angle") || angleNode.getTextContent().isEmpty())
                throw new AutonomousRobotException(TAG, "Element 'angle' missing or empty");

            try {
                angle = Double.parseDouble(angleNode.getTextContent());
            } catch (NumberFormatException nex) {
                throw new AutonomousRobotException(TAG, "Invalid number format in element 'angle'");
            }

            startingPositionData = new StartingPositionData(x, y, angle);
            nextParameterNode = getNextElement(nextParameterNode.getNextSibling());
        }

        // The next optional element in the XML is <vumarks>.
        if ((nextParameterNode != null) && nextParameterNode.getNodeName().equals("vumarks")) {
            NodeList vumarkChildren = nextParameterNode.getChildNodes();
            Node oneVumarkNode;
            for (int i = 0; i < vumarkChildren.getLength(); i++) {
                oneVumarkNode = vumarkChildren.item(i);

                if (oneVumarkNode.getNodeType() != Node.ELEMENT_NODE)
                    continue;

                RobotConstantsFreightFrenzy.SupportedVumark oneVumark = RobotConstantsFreightFrenzy.SupportedVumark.valueOf(oneVumarkNode.getNodeName());
                vumarksOfInterest.add(oneVumark);
            }
            nextParameterNode = getNextElement(nextParameterNode.getNextSibling());
        }

        // Make sure there are no extraneous elements.
        if (nextParameterNode != null)
                  throw new AutonomousRobotException(TAG, "Unrecognized element under <parameters>");

        // Now proceed to the <actions> element of the selected OpMode.
        String actionsPath = opModePath + "/actions";
        Node actionsNode = (Node) xpath.evaluate(actionsPath, document, XPathConstants.NODE);
        if (actionsNode == null)
            throw new AutonomousRobotException(TAG, "Missing <actions> element");

        // Now iterate through the children of the <actions> element of the selected OpMode.
        NodeList actionChildren = actionsNode.getChildNodes();
        Node actionNode;

        RobotXMLElement actionXMLElement;
        for (int i = 0; i < actionChildren.getLength(); i++) {
            actionNode = actionChildren.item(i);

            if (actionNode.getNodeType() != Node.ELEMENT_NODE)
                continue;

            actionXMLElement = new RobotXMLElement((Element) actionNode);
            actions.add(actionXMLElement);
        }

        return new RobotActionDataStandard(logLevel, vumarksOfInterest, startingPositionData,
                actions);
    }

    // Helper method to convert a nested <image_parameters> element into a class.
    public VisionParameters.ImageParameters
    getImageParametersFromXPath(RobotXMLElement pElement, String pPath) throws XPathExpressionException {
        Node ipNode = (Node) xpath.evaluate(pPath, pElement.getRobotXMLElement(), XPathConstants.NODE);
        if (ipNode == null)
            throw new AutonomousRobotException(TAG, "Missing " + pPath + " element");

        if (!ipNode.getNodeName().equals("image_parameters"))
            throw new AutonomousRobotException(TAG, "Expected <image_parameters> element");

        return ImageXML.parseImageParameters(ipNode);
    }

    //**TODO static member in XMLUtils class -> ftcdevcommon.
    private Node getNextElement(Node pNode) {
        Node nd = pNode;
        while (nd != null) {
            if (nd.getNodeType() == Node.ELEMENT_NODE) {
                return nd;
            }
            nd = nd.getNextSibling();
        }
        return null;
    }

    public static class RobotActionDataStandard {
        public final Level logLevel;
        public final List<RobotConstantsFreightFrenzy.SupportedVumark> vumarksOfInterest;
        public final StartingPositionData startingPositionData;
        public final List<RobotXMLElement> actions;

        public RobotActionDataStandard(Level pLogLevel,
                                            List<RobotConstantsFreightFrenzy.SupportedVumark> pVumarks,
                                            StartingPositionData pStartingPositionData,
                                            List<RobotXMLElement> pActions) {
            logLevel = pLogLevel;
            vumarksOfInterest = pVumarks;
            startingPositionData = pStartingPositionData;
            actions = pActions;
        }
    }

    public static class StartingPositionData {

        public final double startingX; // FTC field coordinates
        public final double startingY; // FTC field coordinates
        public final double startingAngle; // with respect to the wall

        public StartingPositionData(double pStartingX, double pStartingY, double pStartingAngle) {
            startingX = pStartingX;
            startingY = pStartingY;
            startingAngle = pStartingAngle;
        }
    }
}