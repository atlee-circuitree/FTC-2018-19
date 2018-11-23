package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class MineralDetector {
    public static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    public enum MineralPosition {
        Left, Right, Center, Unknown;
    }

    private static final String VUFORIA_KEY = "AbPhNsr/////AAABmYWxYoicvU/5i9hkNLOWE6UAf3RvLXp20ph69HXDye36PiCT1d7u9ZHZMshBoDj0PnJSE4nZoLQ2dM6wNOIh9RvXgHzILEZzA3GF8lFFUOvhzVKjxd/GZedz575x4p6iuF3Cnm9LxY4bmHPXYsetQfEaihPglTHrj9ZSmU2WBntj6C8kHUmVw+Skm3QIoMD2o97yaAbOr57NA/bRsA+Ng5JqEzYBpMaicCkXclSqaUIr+N2GZVSKUdNawUS7VkCo6ZAAB/P3xPMkPqyOV5opF8wLbElY6UCONUKabTYImDviq07GLUZ8j5ujQzOUEZ1Q3Es4UL94nbxsm7t/GHogUIiPc95ORT3NtvCyQY3MHlBB";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;
    public Telemetry telemetry;

    HardwareMap hardwareMap = null;


    public void init(HardwareMap hwMapInput, Telemetry telemInput) {
        hardwareMap = hwMapInput;
        telemetry = telemInput;
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
    }

    public MineralPosition GetMineralPosition() {
        MineralPosition position = MineralPosition.Unknown;
        int mineralIndex = 0;
        int detectedSampling = 0;
        int whiteSamples = 0;
        int whiteLeftMost = 1000;
        int goldLeftMost = 1000;

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() >= 2) {

                    for (Recognition recognition : updatedRecognitions) {
                        mineralIndex++;
                        telemetry.addData("Mineral" + mineralIndex, recognition.getHeight());
                        if (recognition.getHeight() >= 80 && recognition.getHeight() < 150) {
                            detectedSampling++;
                            if (recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                                whiteSamples++;
                                if (recognition.getLeft() < whiteLeftMost) {
                                    whiteLeftMost = (int) recognition.getLeft();
                                }
                            } else if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldLeftMost = (int) recognition.getLeft();
                            }
                        }

                    }
                    telemetry.addData("Samples detected", detectedSampling);
                    if (detectedSampling == 2) {
                        if (whiteSamples == 2)
                        {
                            telemetry.addData("GoldPosition", "Left");
                            position = MineralPosition.Left;
                        }
                        else if (whiteSamples == 1) {
                            if (whiteLeftMost < goldLeftMost) {
                                telemetry.addData("GoldPosition", "Right");
                                position = MineralPosition.Right;
                            } else {
                                telemetry.addData("GoldPosition", "Middle");
                                position = MineralPosition.Center;
                            }
                        }
                        else
                        {
                            telemetry.addData("GoldPosition", "Unknown");
                            position = MineralPosition.Unknown;
                        }
                    } else {
                        telemetry.addData("GoldPosition", "Unknown");
                        position = MineralPosition.Unknown;
                    }
                }

            }
        }

        return position;
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public void Shutdown()
    {
        if (tfod != null) {
            tfod.shutdown();
        }
    }
}
