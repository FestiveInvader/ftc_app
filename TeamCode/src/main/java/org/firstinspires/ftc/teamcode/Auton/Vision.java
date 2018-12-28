package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class Vision extends GameSpecificMovement{

    public static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    public static final String VUFORIA_KEY = "Ad2+xnL/////AAABmfD+XRaqfERPmvnHPZzg8b5xA35SCU5QWgaygrkAKRjWp+n" +
            "searSU8Zriv5xsNvOm3cLWfUa7gXGF1h09LWDH6+0QrZ6WVl11ygsh5wTa8IyIZGaPqHG9FjsccPCzNtSPpLZj3vpS4K797weILM" +
            "vElMa4xrSb/xSyn5zWwGEg5H931imaB8yFDkV7LIAxRJgfORqJcrOQ4WVjr6GxEVj2mjNkHNCKF57C1yyY8CYit5BcgDAkz4bosZ" +
            "0jPpvwCks1+trrm5kP+NIj6y49SD+NZh85IUiEITB9ebw49pvA9M8fki18jLYDIexUZ7fnCFj8oBGGnc0CCispwE2ST7ddUDo4" +
            "GmrSSkNLfUrDMjapPpK\n";

    //Start TensorFlow functions (not movement based on TF)
    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
    //End TensorFlow functions (not movement based on TF)

    public int decideFirstSampleheading(){
        int heading;
        if(goldPosition == 1){
            telemetry.addData("On your left", "Marvel reference");
            heading = -40   ;
        }else if (goldPosition == 2){
            telemetry.addData("Center", "Like Shaq");
            heading = 0;
        }else if(goldPosition == 3){
            telemetry.addData("Right", "Like I always am");
            heading = 40;
        }else{
            telemetry.addData("Something is very wrong", "Decide first sample heading function");
            //if this ever shows up, it's most likely that we didn't see the samples in @craterSideSample or something
            heading = 0;
        }
        telemetry.update();
        return heading;
    }

    public double decideSecondSampleheading(){
        double heading = genMovement.getHeading();
        if(goldPosition == 1){
            telemetry.addData("On your left", "Marvel reference");
            heading = 60   ;
        }else if (goldPosition == 2){
            telemetry.addData("Center", "Like Shaq");
            heading = 90;
        }else if(goldPosition == 3){
            telemetry.addData("Right", "Like I always am");
            heading = 120;
        }else{
            telemetry.addData("Something is very wrong", "Decide first sample heading function");
            //if this ever shows up, it's most likely that we didn't see the samples in @craterSideSample or something
            heading = 0;
        }
        telemetry.update();
        return heading;
    }

    public void getGoldPositionTwoMineral(){
        if (goldPosition == 0 && opModeIsActive()) {
            if (tfod != null) {
                tfod.activate();

                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 2) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }
                        if(goldMineralX == -1){
                            telemetry.addData("Gold Mineral Position", "Left");
                            goldPosition = 1;
                        }else if(goldMineralX < silverMineral1X){
                            telemetry.addData("Gold Mineral Position", "Center");
                            goldPosition = 2;
                        }else{
                            telemetry.addData("Gold Mineral Position", "Right");
                            goldPosition = 3;
                        }
                        telemetry.addData("Gold pos", goldPosition);
                        telemetry.addData("Gold X pos", goldMineralX);
                        telemetry.addData("Silver1 X pos", silverMineral1X);
                        telemetry.addData("Silver2 X pos", silverMineral2X);
                        telemetry.update();
                    }
                }
            }
        }
    }
    public void getGoldPositionOneMineral(){
        if (goldPosition == 0 && opModeIsActive()) {
            if (tfod != null) {
                tfod.activate();
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {

                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                            //if gold is found, label it as such
                        } else {
                            //then it's only silver seen, or nothing at all
                        }
                    }
                    if(updatedRecognitions.size() > 0) {
                        //if we have detected any minerals, check and see if we've seen gold
                        //if we have, then if it's on one side of the screen or the other.
                        if (goldMineralX != -1) {
                            //Regardless of if we've seen any other minerals, We can get the gold
                            //position via this
                            if (goldMineralX > 600) {
                                telemetry.addData("Gold Mineral Position", "Right ");
                                goldPosition = 3;
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center");
                                goldPosition = 2;
                            }

                        }
                    }else if(updatedRecognitions.size() == 2){
                        //If we have seen gold, check the gold mineral's pixels X position and see
                        //if it's on the left or right side of the center.
                        if (goldMineralX != -1) {
                            if (goldMineralX > 600) {
                                telemetry.addData("Gold Mineral Position", "Right ");
                                goldPosition = 3;
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center");
                                goldPosition = 2;
                            }

                        }else{
                            //if we see two minerals, and neither are gold, then we know the gold
                            // is on the left side
                            telemetry.addData("Gold Mineral Position", "Left");
                            goldPosition = 1;
                        }
                    }
                    //else if we've seen no minerals, continue looping.
                    telemetry.addData("Gold pos", goldPosition);
                    telemetry.addData("Gold X pos", goldMineralX);
                    telemetry.addData("Silver1 X pos", silverMineral1X);
                    telemetry.addData("Silver2 X pos", silverMineral2X);
                    telemetry.update();
                    telemetry.update();
                    //if one is sensed and it's gold, which side is it on
                    //otherwise, if it's two and neither is gold, left
                }
            }
        }
    }

}
