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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Auton.SensorDrivers.I2CXLv2;


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
    double turningSpeed = .4;

    double potMagicNumber = .01222;

    boolean hangRatchetEngaged = true;
    double hangCamLeftEngagedPos = 1;
    double hangCamLeftUnengagedPos = 0;
    double hangCamRightEngagedPos = 0;
    double hangCamRightUnengagedPos = 1;

    double armScoringRotation = 50;
    double armPVal = .025;
    double armPower;

    double teamMarkerDeploy = -.1;
    double teamMarkerResting = .3;

    int goldPosition = 0;

    public int forward = 1;
    public int reverse = -1;
    public double stayOnHeading = 84.17;

 public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    public ElapsedTime runtime = new ElapsedTime();

    public HangingSystem hangingSystem;
    public Vision vision;
    public GeneralMovement genMovement;
    //@Override
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

        vision.initVuforia();
        telemetry.addData("Vuforia Init'd", true);
        telemetry.update();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            vision.initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        while(!isStarted()) {
            telemetry.addData("IMU", genMovement.getHeading());
            telemetry.addData("FrontLeft Encoder", LeftTop.getCurrentPosition());
            telemetry.addData("If values are good, then run", 1);
            telemetry.addData("If I don't put this here then it'll probably crash", 1);
            telemetry.update();
        }
        ElapsedTime timer = new ElapsedTime();
    }

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
            vision.getGoldPositionOneMineral();
            hangingSystem.unextendHangSlide();
        }
        if(goldPosition == 0){
            //failsafe, so that if this doesn't detect the right two minerals at least we'll still place
            // the team marker and park
            goldPosition = 1;
            //Position 1 is the leftmost mineral
        }
        genMovement.gyroTurn(turningSpeed, vision.decideFirstSampleheading());
        if(goldPosition == 2){
            genMovement.encoderDrive(.5, 12, forward, vision.decideFirstSampleheading(), 2.5);
            genMovement.encoderDrive(.5, 12, reverse, vision.decideFirstSampleheading(), 2.5);
        }else{
            genMovement.encoderDrive(.5, 20, forward, vision.decideFirstSampleheading(), 2.5);
            genMovement.encoderDrive(.5, 20, reverse, vision.decideFirstSampleheading(), 2.5);
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
            vision.getGoldPositionOneMineral();
            hangingSystem.unextendHangSlide();
        }
        if(goldPosition == 0){
            //failsafe, so that if this doesn't detect the right two minerals at least we'll still place
            // the team marker and park
            goldPosition = 1;
            //Position 1 is the leftmost mineral
        }
        genMovement.gyroTurn(turningSpeed, vision.decideFirstSampleheading());
        if(goldPosition == 2){
            genMovement.encoderDrive(.5, 12, forward, vision.decideFirstSampleheading(), 2.5);
            genMovement.encoderDrive(.5, 12, reverse, vision.decideFirstSampleheading(), 2.5);
        }else{
            genMovement.encoderDrive(.5, 18, forward, vision.decideFirstSampleheading(), 2.5);
            genMovement.encoderDrive(.5, 18, reverse, vision.decideFirstSampleheading(), 2.5);
        }
    }


    public void depotSideDeployMarker(){
        genMovement.gyroTurn(turningSpeed, -90);
        genMovement.encoderDrive(.5, 12, forward, stayOnHeading, 2);
        genMovement.gyroTurn(turningSpeed, -45);
        genMovement.encoderDrive(.35, 36, forward, stayOnHeading, 4);
        //just hit the wall
        genMovement.encoderDrive(.2, 3, reverse, stayOnHeading, 2);
        genMovement.gyroTurn(turningSpeed, 45);
        genMovement.goToDistance(.35, 50, FrontDistance, 3, 3);
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
        genMovement.goToDistance(.35, 100, FrontDistance,5,3);
        genMovement.gyroTurn(turningSpeed, -30);
        genMovement.encoderDrive(.75, 18, reverse, stayOnHeading, 3);
        genMovement.gyroTurn(turningSpeed, 18);
        genMovement.encoderDrive(.75, 20, reverse, stayOnHeading, 3);
        genMovement.gyroTurn(turningSpeed, 90);
        genMovement.encoderDrive(.2, 4, reverse, stayOnHeading, 1.25);
        genMovement.gyroTurn(turningSpeed, vision.decideSecondSampleheading());
        if(goldPosition == 2){
            genMovement.encoderDrive(.5, 16, forward, stayOnHeading, 2.5);
        }else{
            genMovement.encoderDrive(.5, 24, forward, stayOnHeading, 2.5);
        }
    }
    public void driveFromDepot(){}


    /*
    Once we're unlatched, turn to the right until we see two minerals
    then, based off of those two minerals, turn to a position
    go forward x inches
    go backward x inches
    turn left
    */

    public void endAuto(){
        //telemetry for autonomous testing to see any factors that may have went wrong
        TeamMarker.setPosition(teamMarkerResting);
        while(opModeIsActive() && hangingSystem.hangSlideIsExtended()){

        }
        telemetry.addData("No Glyphs", "Cuz that was last year");
        telemetry.update();
    }

}