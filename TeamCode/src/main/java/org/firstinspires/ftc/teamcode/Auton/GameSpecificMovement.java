package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.I2CXLv2;
import org.firstinspires.ftc.teamcode.Auton.GeneralMovement;
import org.firstinspires.ftc.teamcode.Auton.HangingSystem;

import java.util.List;


//@Author Eric Adams, Team 8417 'Lectric Legends

public class GameSpecificMovement extends LinearOpMode {
    // This section declares hardware for the program, such as Motors, servos and sensors
    // Declare Motors
    public DcMotor LeftTop = null;
    public DcMotor LeftBottom = null;
    public DcMotor RightTop = null;
    public DcMotor RightBottom = null;
    public DcMotor HangingSlide = null;
    public DcMotor ArmTop = null;
    public DcMotor ArmBottom = null;
    public DcMotor ArmSlide = null;

    //public DcMotor ArmTop = null;
    //public DcMotor ArmMid = null;
    //public DcMotor ArmBottom = null;
    //public DcMotor ArmSlide = null;


    // Declare Servos
    public CRServo IntakeLeft;
    public CRServo IntakeRight;
    public Servo HangCamLeft;
    public Servo HangCamRight;
    public Servo TeamMarker;
    //public CRServo IntakeLeft;                  // Rev SRS
    //public Servo PTOShifterLeft;                      // Rev SRS
    //public Servo PTOShifterRight;
    public DigitalChannel HangSlideLimit;
    public AnalogInput ArmPot;
    public BNO055IMU IMU;
    public I2CXLv2 FrontDistance;

    // Variables used  in functions
    double CountsPerRev = 537.6;    // Andymark NeveRest 20 encoder counts per revolution
    double GearRatio = 56/42;
    double WheelDiameterInches = 3.0;     // For figuring circumference
    double CountsPerInch = ((CountsPerRev / ((WheelDiameterInches * 3.1415))/GearRatio));

    double rev40TicksPerRev = 1120;
    double hangingPulleyDiameter = 1.1;
    double hangingGearRatio = 60/40; //Total ratio of hang / motor reduction gives ticks per rot
    double ticksPerHangingRev = rev40TicksPerRev*hangingGearRatio;

    double ticksPerHangingInch =  (ticksPerHangingRev/(hangingPulleyDiameter * 3.1415));

    double HEADING_THRESHOLD = 2;      // As tight as we can make it with an integer gyro
    double P_TURN_COEFF = .2;     // Larger is more responsive, but also less stable
    double P_DRIVE_COEFF = .15;     // Larger is more responsive, but also less stable
    public double turningSpeed = .4;

    double potMagicNumber = .01222;


    double teamMarkerDeploy = -.1;
    double teamMarkerResting = .3;

    int goldPosition = 0;

    public int forward = 1;
    public int reverse = -1;
    public double stayOnHeading = 84.17;

    public static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    public static final String VUFORIA_KEY = "Ad2+xnL/////AAABmfD+XRaqfERPmvnHPZzg8b5xA35SCU5QWgaygrkAKRjWp+n" +
            "searSU8Zriv5xsNvOm3cLWfUa7gXGF1h09LWDH6+0QrZ6WVl11ygsh5wTa8IyIZGaPqHG9FjsccPCzNtSPpLZj3vpS4K797weILM" +
            "vElMa4xrSb/xSyn5zWwGEg5H931imaB8yFDkV7LIAxRJgfORqJcrOQ4WVjr6GxEVj2mjNkHNCKF57C1yyY8CYit5BcgDAkz4bosZ" +
            "0jPpvwCks1+trrm5kP+NIj6y49SD+NZh85IUiEITB9ebw49pvA9M8fki18jLYDIexUZ7fnCFj8oBGGnc0CCispwE2ST7ddUDo4" +
            "GmrSSkNLfUrDMjapPpK\n";

    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    public GeneralMovement genMovement = new GeneralMovement();
    public HangingSystem hangingSystem = new HangingSystem();

    public ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // This section gets the hardware maps
        telemetry.addData("Status", "Startiiiiiiii  ng Init");
        telemetry.update();
        // This section gets the hardware maps
        LeftTop = hardwareMap.dcMotor.get("LeftTop");
        LeftBottom = hardwareMap.dcMotor.get("LeftBottom");
        RightTop = hardwareMap.dcMotor.get("RightTop");
        RightBottom = hardwareMap.dcMotor.get("RightBottom");
        HangingSlide = hardwareMap.dcMotor.get("HangingSlide");
        ArmTop = hardwareMap.dcMotor.get("ArmTop");
        ArmBottom = hardwareMap.dcMotor.get("ArmBottom");
        ArmSlide = hardwareMap.dcMotor.get("ArmSlide");

        IntakeLeft = hardwareMap.crservo.get("IntakeLeft");
        IntakeRight = hardwareMap.crservo.get("IntakeRight");
        HangCamLeft = hardwareMap.servo.get("HangCamLeft");
        HangCamRight = hardwareMap.servo.get("HangCamRight");
        TeamMarker = hardwareMap.servo.get("TeamMarker");

        HangSlideLimit = hardwareMap.get(DigitalChannel.class, "HangSlideLimit");
        HangSlideLimit.setMode(DigitalChannel.Mode.INPUT);

        ArmPot = hardwareMap.analogInput.get("ArmPot");
        FrontDistance = hardwareMap.get(I2CXLv2.class, "FrontDistance");

        LeftTop.setDirection(DcMotorSimple.Direction.FORWARD);
        LeftBottom.setDirection(DcMotorSimple.Direction.REVERSE);
        RightTop.setDirection(DcMotorSimple.Direction.REVERSE);
        RightBottom.setDirection(DcMotorSimple.Direction.FORWARD);
        HangingSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ArmSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IntakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Start Init IMU
        BNO055IMU.Parameters Bparameters = new BNO055IMU.Parameters();
        Bparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        Bparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        Bparameters.calibrationDataFile = "BNO055IMUCalibration.json";
        Bparameters.loggingEnabled = true;
        Bparameters.loggingTag = "IMU";
        Bparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        IMU = hardwareMap.get(BNO055IMU.class, "IMU");
        IMU.initialize(Bparameters);
        // End Init IMU
        telemetry.addData("IMU Init'd", true);
        telemetry.update();
        //vuforiaHardware = new VuforiaHardware();
        //vuforiaHardware.Init(hardwareMap);

        initVuforia();
        telemetry.addData("Vuforia Init'd", true);
        telemetry.update();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        while(!opModeIsActive()&&!isStopRequested()) {
            telemetry.addData("IMU", genMovement.getHeading());
            telemetry.addData("FrontLeft Encoder", LeftTop.getCurrentPosition());
            telemetry.addData("If values are good, then run", 1);
            telemetry.addData("If I don't put the not !waitforstart or whatever here then it'll probably crash", 1);
            telemetry.update();
        }
        ElapsedTime timer = new ElapsedTime();
    }



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

    //Start Rover Ruckus specific movement and logic functions
    public void craterSideSample(){
        /*//should come immediately after unlatching
        //for the most part this should be able to be copy/pasted to the depotSideSample, though a few changes
        //for the team marker may have to be made.
        gyroTurn(turningSpeed, 0);
        encoderDrive(.35, 2, 1, stayOnHeading, 2);
        while(goldPosition == 0 && getHeading() < 45 &&opModeIsActive()){
            pivot(.425, 1);
            getGoldPositionTwoMineral();
            unextendHangSlide();
        }
        if(goldPosition == 0){
            //failsafe, so that if this doesn't detect the right two minerals at least we'll still place
            // the team marker and park
            goldPosition = 1;
        }
        gyroTurn(turningSpeed, decideFirstSampleheading());
        if(goldPosition == 2){
            encoderDrive(.35, 12, forward, stayOnHeading, 5);
        }else{
            encoderDrive(.35, 24, forward, stayOnHeading, 5);
        }
*/
        ElapsedTime elapsedTime = new ElapsedTime();
//should come immediately after unlatching
        //for the most part this should be able to be copy/pasted to the depotSideSample, though a few changes
        //for the team marker may have to be made.
        genMovement.gyroTurn(turningSpeed, 0);
        genMovement.encoderDrive(.35, 2, 1, stayOnHeading, 2);
        genMovement.gyroTurn(turningSpeed, 15);
        while(goldPosition == 0 && elapsedTime.seconds() < 3 && opModeIsActive()){
            getGoldPositionOneMineral();
            hangingSystem.unextendHangSlide();
        }
        if(goldPosition == 0){
            //failsafe, so that if this doesn't detect the right two minerals at least we'll still place
            // the team marker and park
            goldPosition = 1;
            //Position 1 is the leftmost mineral
        }
        genMovement.gyroTurn(turningSpeed, decideFirstSampleheading());
        if(goldPosition == 2){
            genMovement.encoderDrive(.5, 12, forward, decideFirstSampleheading(), 2.5);
            genMovement.encoderDrive(.5, 12, reverse, decideFirstSampleheading(), 2.5);
        }else{
            genMovement.encoderDrive(.5, 20, forward, decideFirstSampleheading(), 2.5);
            genMovement.encoderDrive(.5, 20, reverse, decideFirstSampleheading(), 2.5);
        }
    }
    public void depotSideSample(){
        ElapsedTime elapsedTime = new ElapsedTime();
//should come immediately after unlatching
        //for the most part this should be able to be copy/pasted to the depotSideSample, though a few changes
        //for the team marker may have to be made.
        genMovement.gyroTurn(turningSpeed, 0);
        genMovement.encoderDrive(.35, 2, 1, stayOnHeading, 2);
        genMovement.gyroTurn(turningSpeed, 15);
        while(goldPosition == 0 && elapsedTime.seconds() < 3 && opModeIsActive()){
            getGoldPositionOneMineral();
            hangingSystem.unextendHangSlide();
        }
        if(goldPosition == 0){
            //failsafe, so that if this doesn't detect the right two minerals at least we'll still place
            // the team marker and park
            goldPosition = 1;
            //Position 1 is the leftmost mineral
        }
        genMovement.gyroTurn(turningSpeed, decideFirstSampleheading());
        if(goldPosition == 2){
            genMovement.encoderDrive(.5, 12, forward, decideFirstSampleheading(), 2.5);
            genMovement.encoderDrive(.5, 12, reverse, decideFirstSampleheading(), 2.5);
        }else{
            genMovement.encoderDrive(.5, 18, forward, decideFirstSampleheading(), 2.5);
            genMovement.encoderDrive(.5, 18, reverse, decideFirstSampleheading(), 2.5);
        }
    }
    public void driveFromCraterAfterSampleToNearDepot(){
        genMovement.encoderDrive(.5, 24, forward, stayOnHeading, 3);
        genMovement.gyroTurn(turningSpeed, -135);//turn to the left, facing the depot
        genMovement.goToDistance(.35, 55, 3, 2);
    }


    public void depotSideDeployMarker(){
        genMovement.gyroTurn(turningSpeed, -90);//At this point we'll be facing the other alliances crater-ish
        genMovement.encoderDrive(.5, 12, forward, stayOnHeading, 2);
        genMovement.gyroTurn(turningSpeed, -45);//face the near non-alliance wall
        genMovement.encoderDrive(.35, 36, forward, stayOnHeading, 4);//just hit the wall
        genMovement.encoderDrive(.2, 3, reverse, stayOnHeading, 2);//back away from the wall for turning clearance
        genMovement.gyroTurn(turningSpeed, 45);//turn towards the depot
        genMovement.goToDistance(.35, 50, 3, 3); //drive to the edge of the depot
    }

    public void deployTeamMarker(){
        TeamMarker.setPosition(teamMarkerDeploy);
    }
    public void depotTurnToFarCrater(){
        //This will drive to the other alliance's side's crater, to be out of the way of our partner's team marker.
        //We should also have a version that goes to our side, if our alliance partner also scores in the lander (so that
        // we get a little bit of extra time for cycles.
        genMovement.gyroTurn(turningSpeed, -45);
        genMovement.encoderDrive(.425, 18, forward, stayOnHeading, 2);
        double thisHeading = genMovement.getHeading();
        genMovement.encoderDrive(.35, 2.5, reverse, stayOnHeading, 1.5);
        TeamMarker.setPosition(teamMarkerResting);
        double turningHeading = -thisHeading + 91;
        genMovement.gyroTurn(turningSpeed, turningHeading);
    }
    public void depotTurnToCloseCrater(){
        //This will drive to the other alliance's side's crater, to be out of the way of our partner's team marker.
        //We should also have a version that goes to our side, if our alliance partner also scores in the lander (so that
        // we get a little bit of extra time for cycles.
        genMovement.gyroTurn(turningSpeed, 45);
        genMovement.encoderDrive(.425, 18, forward, stayOnHeading, 2);
        double thisHeading = genMovement.getHeading();
        genMovement.encoderDrive(.35, 2, reverse, stayOnHeading, 1.5);
        TeamMarker.setPosition(teamMarkerResting);
        double turningHeading = -thisHeading - 89;
        genMovement.gyroTurn(turningSpeed, turningHeading);
    }
    public void depotSideDoubleSample(){
        genMovement.goToDistance(.35, 100, 5,3);
        genMovement.gyroTurn(turningSpeed, -30);
        genMovement.encoderDrive(.75, 18, reverse, stayOnHeading, 3);
        genMovement.gyroTurn(turningSpeed, 18);
        genMovement.encoderDrive(.75, 20, reverse, stayOnHeading, 3);
        genMovement.gyroTurn(turningSpeed, 90);
        genMovement.encoderDrive(.2, 4, reverse, stayOnHeading, 1.25);
        genMovement.gyroTurn(turningSpeed, decideSecondSampleheading());
        if(goldPosition == 2){
            genMovement.encoderDrive(.5, 16, forward, stayOnHeading, 2.5);
        }else{
            genMovement.encoderDrive(.5, 24, forward, stayOnHeading, 2.5);
        }
    }
    public void craterSideParkArmInCrater(){
        genMovement.goToDistance(.5, 100, 5,4);
        genMovement.gyroTurn(turningSpeed, -120);
        genMovement.encoderDrive(.75, 18, reverse, stayOnHeading, 3);
        genMovement.gyroTurn(turningSpeed, -72);
        genMovement.encoderDrive(.75, 20, reverse, stayOnHeading, 3);
        genMovement.gyroTurn(turningSpeed, 0);
    }
    public void driveFromDepot(){}







    public void goToDistance(double targetSpeed, double distance, double timeout, int tolerance){
        double startTime = runtime.seconds();
        double maxTime = startTime + timeout;
        double startHeading = genMovement.getHeading();
        boolean foundTarget = false;
        int ThisLoopDistance;
        while (opModeIsActive() && !foundTarget && maxTime - runtime.seconds() > .1) {
            ThisLoopDistance = FrontDistance.getDistance();
            double error = distance - ThisLoopDistance;
            int Direction = (int) -Range.clip(error, -1, 1);
            if(ThisLoopDistance > 500 || ThisLoopDistance < 21){
                genMovement.gyroDrive(startHeading, Range.clip(Math.abs(error/100), .25, targetSpeed), Direction);
                //sensor val is bad, stop bot so it doesn't go too far
            }else if(ThisLoopDistance > distance + tolerance || ThisLoopDistance < distance - tolerance){
                genMovement.gyroDrive(startHeading, Range.clip(Math.abs(error/100), .25, targetSpeed), Direction);
            }else{
                genMovement.stopDriveMotors();
                foundTarget = true;
            }
            telemetry.addData("Distance", ThisLoopDistance);
            telemetry.addData("error", error);
            telemetry.addData("speed", LeftTop.getPower());
            telemetry.update();
        }
        genMovement.stopDriveMotors();
    }

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

    /*
    Once we're unlatched, turn to the right until we see two minerals
    then, based off of those two minerals, turn to a position
    go forward x inches
    go backward x inches
    turn left
    */

    public void endAuto(boolean endWithArmUp){
        //telemetry for autonomous testing to see any factors that may have went wrong
        TeamMarker.setPosition(teamMarkerResting);
        if(endWithArmUp) {
            while (opModeIsActive() && hangingSystem.hangSlideIsExtended()) {}
        }else{
            while (opModeIsActive()) {
                hangingSystem.unextendHangSlide();
                hangingSystem.putMineralArmDown();
            }
        }
        telemetry.addData("No Glyphs", "Cuz that was last year");
        telemetry.update();
    }










}