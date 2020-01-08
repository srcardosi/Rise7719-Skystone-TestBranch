package org.firstinspires.ftc.teamcode.MkI.Subsystems.VisionTargeting;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/*
 * Created by Sean Cardosi on 9/22/2019.
 *
 * Susbsystem used to detect Skystones and Stones using TensorFlow from the phone's built-in camera.
 */
public class SkystoneDetectionPhone {

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    public SkystoneDetectionPhone(final HardwareMap _hardwareMap, final Telemetry _telemetry) {
        hardwareMap = _hardwareMap;
        telemetry = _telemetry;
    }

    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/Skystone.tflite"; //For OpenRC, loaded from internal storage to reduce APK size
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
//    boolean isSkystone = false;
    private static final String VUFORIA_KEY = "AUbWH2r/////AAABmTqIGJpUgkb9j4jexPb0CKYmpPjDhVyV5bE2RL866jD2AKG/pqN0P8mPlybMo3P0xERT+mK0uW04FHPso8OcJDLER7gW6Rjnv49Yzc7ks3zkCGtwmHx/sqInqUl4i2jlHTFiW8qYnAf/iOJ0O2jO7j8UjOuurbpGT+3iGwRprWQFe7/Wb6k08A1tMIwvDKgU3g+PudWyfefPeo2Oo3PYzIiGu+KlswOR26Jn3jRSGmlin3JrfLkvmV7AmTaFWGwb876eR21A5EP40EIBk8E9nDuJcVB60q9R7nnBvf/qjsSuUwKQWtn9xTBzWSIhEzXetUY5PMxckiugQLWHp7YAkrM7SGNte2JrUk9XslYRqV4z";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public void init() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /*
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         */
        if (tfod != null) {
            tfod.activate();
        }

        /* Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
    }


    public boolean TFdetect(boolean isSkystone) {
        isSkystone = false;

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    if (recognition.getLabel() == LABEL_SECOND_ELEMENT) {
                        isSkystone = true;
                    } else {
                        isSkystone = false;
                    }
                }
                telemetry.update();
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
        return isSkystone;
    }

    /*
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia () {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /*
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod () {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}