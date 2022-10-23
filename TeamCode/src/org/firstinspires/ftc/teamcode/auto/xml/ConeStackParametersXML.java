package org.firstinspires.ftc.teamcode.auto.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.intellij.RobotLogCommon;
import org.firstinspires.ftc.teamcode.auto.vision.ConeStackParameters;
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
// information needed for our OpenCV methods to recognize a stack
// of cones during Autonomous.
public class ConeStackParametersXML {
    public static final String TAG = ConeStackParametersXML.class.getSimpleName();
    private static final String CONE_STACK_FILE_NAME = "ConeStackParameters.xml";

    private final Document document;
    private final XPath xpath;

    public ConeStackParametersXML(String pXMLDir) {
        try {
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            dbFactory.setIgnoringComments(true);

            // ONLY works with a validating parser (DTD or schema)
            // dbFactory.setIgnoringElementContentWhitespace(true);
            // Not supported in Android Studio dbFactory.setXIncludeAware(true);

            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
            document = dBuilder.parse(new File(pXMLDir + CONE_STACK_FILE_NAME));
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

    public ConeStackParameters getConeStackParameters() throws XPathExpressionException {
        XPathExpression expr;
        VisionParameters.GrayParameters grayParameters;
        VisionParameters.HSVParameters hsvParameters;

        // Point to the first node.
        RobotLogCommon.d(TAG, "Parsing XML cone_stack_parameters");

        expr = xpath.compile("//cone_stack_parameters");
        Node cone_stack_parameters_node = (Node) expr.evaluate(document, XPathConstants.NODE);
        if (cone_stack_parameters_node == null)
            throw new AutonomousRobotException(TAG, "Element '//cone_stack_parameters' not found");

        // Point to <grayscale>
        Node grayscale_node = cone_stack_parameters_node.getFirstChild();
        grayscale_node = getNextElement(grayscale_node);
        if ((grayscale_node == null) || !grayscale_node.getNodeName().equals("grayscale"))
            throw new AutonomousRobotException(TAG, "Element 'grayscale' not found");

        // Point to <gray_parameters>
        Node gray_node = grayscale_node.getFirstChild();
        Node gray_parameters_node = getNextElement(gray_node);
        if ((gray_parameters_node == null) || !gray_parameters_node.getNodeName().equals("gray_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'gray_parameters' not found");

        grayParameters = ImageXML.parseGrayParameters(gray_parameters_node);
        return new ConeStackParameters(grayParameters);
    }

    //**TODO THIS belongs in ftcdevcommon for IntelliJ and Android -> XMLUtils
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

