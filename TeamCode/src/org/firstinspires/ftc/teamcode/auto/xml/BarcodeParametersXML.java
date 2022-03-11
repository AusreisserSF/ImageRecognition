package org.firstinspires.ftc.teamcode.auto.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.intellij.RobotLogCommon;
import org.firstinspires.ftc.teamcode.auto.vision.VisionParameters;
import org.firstinspires.ftc.teamcode.auto.vision.BarcodeParameters;
import org.w3c.dom.Document;
import org.w3c.dom.Node;
import org.xml.sax.SAXException;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.*;
import java.io.File;
import java.io.IOException;

// Class whose job it is to read an XML file that contains all of the information
// needed for our OpenCV methods to recognize our Freight Frenzy Team Scoring
// Element in the barcode during Autonomous.
public class BarcodeParametersXML {
    public static final String TAG = "BarcodeParametersXML";
    private static final String BCP_FILE_NAME = "BarcodeParameters.xml";

    private final Document document;
    private final XPath xpath;

    public BarcodeParametersXML(String pXMLDir) {
        try {
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            dbFactory.setIgnoringComments(true);

            // ONLY works with a validating parser (DTD or schema)
            // dbFactory.setIgnoringElementContentWhitespace(true);
            // Not supported in Android Studio dbFactory.setXIncludeAware(true);

            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
            document = dBuilder.parse(new File(pXMLDir + BCP_FILE_NAME));
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

    public BarcodeParameters getBarcodeParameters() throws XPathExpressionException {
        XPathExpression expr;
        VisionParameters.GrayParameters grayParameters;
        VisionParameters.HSVParameters hsvParameters;

        // Point to the first node.
        RobotLogCommon.d(TAG, "Parsing XML barcode_parameters");

        expr = xpath.compile("//barcode_parameters");
        Node barcode_parameters_node = (Node) expr.evaluate(document, XPathConstants.NODE);
        if (barcode_parameters_node == null)
            throw new AutonomousRobotException(TAG, "Element '//barcode_parameters' not found");

        // Point to <gray_parameters>
        Node gray_node = barcode_parameters_node.getFirstChild();
        Node gray_parameters_node = getNextElement(gray_node);
        if ((gray_parameters_node == null) || !gray_parameters_node.getNodeName().equals("gray_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'gray_parameters' not found");

        grayParameters = ImageXML.parseGrayParameters(gray_parameters_node);

        // Point to <hsv_parameters>
        Node hsv_node = gray_parameters_node.getNextSibling();
        Node hsv_parameters_node = getNextElement(hsv_node);
        if ((hsv_parameters_node == null) || !hsv_parameters_node.getNodeName().equals("hsv_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'hsv_parameters' not found");

        hsvParameters = ImageXML.parseHSVParameters(hsv_parameters_node);

        // Point to <criteria>
        Node criteria_node = hsv_parameters_node.getNextSibling();
        criteria_node = getNextElement(criteria_node);
        if ((criteria_node == null) || !criteria_node.getNodeName().equals("criteria"))
            throw new AutonomousRobotException(TAG, "Element 'criteria' not found");

        // Point to <min_white_pixels>
        Node pixel_count_node = criteria_node.getFirstChild();
        pixel_count_node = getNextElement(pixel_count_node);
        if ((pixel_count_node == null) || !pixel_count_node.getNodeName().equals("min_white_pixels") || pixel_count_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'criteria/min_white_pixels' missing or empty");

        int minWhitePixels;
        try {
            minWhitePixels = Integer.parseInt(pixel_count_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'criteria/min_pixel_count'");
        }

        return new BarcodeParameters(grayParameters, hsvParameters, minWhitePixels);
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

