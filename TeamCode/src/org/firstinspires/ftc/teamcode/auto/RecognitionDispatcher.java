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
import org.firstinspires.ftc.ftcappcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcappcommon.RobotLogCommon;
import org.firstinspires.ftc.teamcode.auto.vision.FileImage;
import org.firstinspires.ftc.teamcode.auto.vision.RingParameters;
import org.firstinspires.ftc.teamcode.auto.vision.RingRecognition;
import org.firstinspires.ftc.teamcode.auto.vision.RingReturn;
import org.firstinspires.ftc.teamcode.auto.xml.RingParametersXML;

import java.io.FileInputStream;
import java.io.InputStream;

public class RecognitionDispatcher extends Application {

    private static final String TAG = "RingRecognition";

    public static final double FIELD_WIDTH = 410;
    public static final double FIELD_HEIGHT = 300;

    public static final double NOTIFICATION_TEXT_POSITION_Y = (FIELD_HEIGHT / 2) + 20;
    public static final double NOTIFICATION_TEXT_POSITION_X = 10;
    public static final Font NOTIFICATION_TEXT_FONT = Font.font("Comic Sans MS", FontWeight.BOLD, 24);

    // See https://stackoverflow.com/questions/21729362/trying-to-call-a-javafx-application-from-java-nosuchmethodexception
    //!! Default constructor is required. This is the one that launches calls.
    public RecognitionDispatcher() {
        RobotLogCommon.i(TAG, "Starting ring recognition");
    }

    @Override
    public void start(Stage pStage) throws Exception {
        // Parent root = FXMLLoader.load(getClass().getResource("simulator.fxml"));
        Pane field = new Pane();

        // Read the parameters for ring recognition from the xml file.
        RingParametersXML ringParametersXML = new RingParametersXML();
        RingParameters ringParameters = ringParametersXML.getRingParameters();
        RingRecognition ringRecognition = new RingRecognition();

        // Call the OpenCV subsystem.
        FileImage fileImage = new FileImage(ringParameters.imageParameters.file_name);
        RingReturn ringReturn = ringRecognition.findGoldRings(fileImage, ringParameters);
        if (ringReturn.fatalComputerVisionError)
            throw new AutonomousRobotException(TAG, "Error in computer vision subsystem");

        RobotLogCommon.d(TAG, "Found Target Zone " + ringReturn.targetZone);

        // Display the image in the Pane.

        //String imageFilename = "\"" + ringParameters.imageParameters.file_name + "\"";
        //Image image = new Image(imageFilename);
        InputStream stream = new FileInputStream(ringParameters.imageParameters.file_name);
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
    }

}