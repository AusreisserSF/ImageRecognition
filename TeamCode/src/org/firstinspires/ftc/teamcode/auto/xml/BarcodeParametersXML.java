package org.firstinspires.ftc.teamcode.auto.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
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
        ImageXML imageXml = new ImageXML();
        XPathExpression expr;
        VisionParameters.GrayParameters grayParameters;

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

        grayParameters = imageXml.parseGrayParameters(gray_parameters_node);

        return new BarcodeParameters(grayParameters);
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

