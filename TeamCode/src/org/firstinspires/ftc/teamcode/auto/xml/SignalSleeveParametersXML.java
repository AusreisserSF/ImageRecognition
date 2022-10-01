package org.firstinspires.ftc.teamcode.auto.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.intellij.RobotLogCommon;
import org.firstinspires.ftc.teamcode.auto.vision.RealSenseParameters;
import org.firstinspires.ftc.teamcode.auto.vision.SignalSleeveParameters;
import org.firstinspires.ftc.teamcode.auto.vision.VisionParameters;
import org.w3c.dom.Document;
import org.w3c.dom.Node;
import org.xml.sax.SAXException;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.*;
import java.io.File;
import java.io.IOException;

// Class whose job it is to read an XML file that contains all of the
// information needed for our OpenCV methods to recognize our custom
// Signal Sleeve during Autonomous.
public class SignalSleeveParametersXML {
    public static final String TAG = SignalSleeveParametersXML.class.getSimpleName();
    private static final String SIGNAL_SLEEVE_FILE_NAME = "SignalSleeveParameters.xml";

    private final Document document;
    private final XPath xpath;

    public SignalSleeveParametersXML(String pXMLDir) {
        try {
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            dbFactory.setIgnoringComments(true);

            // ONLY works with a validating parser (DTD or schema)
            // dbFactory.setIgnoringElementContentWhitespace(true);
            // Not supported in Android Studio dbFactory.setXIncludeAware(true);

            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
            document = dBuilder.parse(new File(pXMLDir + SIGNAL_SLEEVE_FILE_NAME));
            XPathFactory xpathFactory = XPathFactory.newInstance();
            xpath = xpathFactory.newXPath();

        } catch (ParserConfigurationException pex) {
            throw new AutonomousRobotException(TAG, "DOM parser Exception " + pex.getMessage());
        } catch (SAXException sx) {
            throw new AutonomousRobotException(TAG, "SAX Exception " + sx.getMessage());
        } catch (IOException iex) {
            throw new AutonomousRobotException(TAG, "IOException " + iex.getMessage());
        }
    }

    public SignalSleeveParameters getSignalSleeveParameters() throws XPathExpressionException {
        XPathExpression expr;
        VisionParameters.GrayParameters grayParameters;

        // Point to the first node.
        RobotLogCommon.d(TAG, "Parsing XML signal_sleeve_parameters");

        expr = xpath.compile("//signal_sleeve_parameters");
        Node signal_sleeve_parameters_node = (Node) expr.evaluate(document, XPathConstants.NODE);
        if (signal_sleeve_parameters_node == null)
            throw new AutonomousRobotException(TAG, "Element '//signal_sleeve_parameters' not found");

        // Point to <gray_parameters>
        Node gray_node = signal_sleeve_parameters_node.getFirstChild();
        Node gray_parameters_node = getNextElement(gray_node);
        if ((gray_parameters_node == null) || !gray_parameters_node.getNodeName().equals("gray_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'gray_parameters' not found");

        grayParameters = ImageXML.parseGrayParameters(gray_parameters_node);

        // Point to <criteria>
        Node criteria_node = gray_parameters_node.getNextSibling();
        criteria_node = getNextElement(criteria_node);
        if ((criteria_node == null) || !criteria_node.getNodeName().equals("criteria"))
            throw new AutonomousRobotException(TAG, "Element 'criteria' not found");

        //## An all-black Signal Sleeve indicates location 1 so no minimum number
        // of white pixels is needed.
        // Point to <min_white_pixels_location_2>
        Node pixel_count_2_node = criteria_node.getFirstChild();
        pixel_count_2_node = getNextElement(pixel_count_2_node);
        if ((pixel_count_2_node == null) || !pixel_count_2_node.getNodeName().equals("min_white_pixels_location_2") || pixel_count_2_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'criteria/min_white_pixels_location_2' missing or empty");

        int minWhitePixelsLocation2;
        try {
            minWhitePixelsLocation2 = Integer.parseInt(pixel_count_2_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'criteria/min_white_pixels_location_2'");
        }

        // Point to <min_white_pixels_location_3>
        Node pixel_count_3_node = pixel_count_2_node.getNextSibling();
        pixel_count_3_node = getNextElement(pixel_count_3_node);
        if ((pixel_count_3_node == null) || !pixel_count_3_node.getNodeName().equals("min_white_pixels_location_3") || pixel_count_3_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'criteria/min_white_pixels_location_3' missing or empty");

        int minWhitePixelsLocation3;
        try {
            minWhitePixelsLocation3 = Integer.parseInt(pixel_count_3_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'criteria/min_white_pixels_location_3'");
        }

        return new SignalSleeveParameters(grayParameters, minWhitePixelsLocation2, minWhitePixelsLocation3);
    }

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
}

