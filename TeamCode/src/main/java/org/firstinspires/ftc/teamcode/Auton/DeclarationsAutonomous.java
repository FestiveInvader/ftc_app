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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

//@Author Eric Adams, Team 8417 'Lectric Legends

public class DeclarationsAutonomous extends LinearOpMode {
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

    // Declare Servos
    public CRServo IntakeLeft;
    public CRServo IntakeRight;
    public CRServo TeamMarker;
    public Servo HangCamLeft;
    public Servo HangCamRight;
    public Servo IntakeFlapLeft;

    public DigitalChannel HangSlideLimit;
    public AnalogInput ArmPot;
    public BNO055IMU IMU;
    public I2CXLv2 FrontDistance;

    // Variables used  in functions
    double CountsPerRev = 537.6;    // Andymark NeveRest 20 encoder counts per revolution
    double GearRatio = 56/42;
    double WheelDiameterInches = 3.0;     // For figuring circumference
    double CountsPerInch = ((CountsPerRev / ((WheelDiameterInches * 3.1415))/GearRatio));

    double hangingMotorCountsPerInch = 537.6;
    double hangingPulleyDiameter = .75;
    double hangingGearRatio = 60/40;
    double ticksPerHangingRev = hangingMotorCountsPerInch *hangingGearRatio;

    double mineralArmSpoolDiameter = 1;
    double mineralArmMotorCountsPerInch = 1120;
    double ticksToExtendMineralArmInch = (mineralArmMotorCountsPerInch /(mineralArmSpoolDiameter * 3.1415));

    double ticksPerHangingInch =  (ticksPerHangingRev/(hangingPulleyDiameter * 3.1415));

    double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    double P_TURN_COEFF = .0021
            ;     // Larger is more responsive, but also less stable .0035 is best so far
    double P_DRIVE_COEFF = .15;     // Larger is more responsive, but also less stable
    public double turningSpeed = .4;

    double potMagicNumber = .01222;

    boolean hangRatchetEngaged = true;
    double hangCamLeftEngagedPos = 1;
    double hangCamLeftUnengagedPos = 0;
    double hangCamRightEngagedPos = 0;
    double hangCamRightUnengagedPos = 1;

    double armScoringRotation = 65;
    double armDownRotation = 110;
    double armPVal = .02;
    double armPower;

    public double teamMarkerDeploy = -.9;
    public double teamMarkerResting = .3;

    double intakeFlapLeftOpen = .8;
    double intakeFlapLeftClosed = 0;


    int goldPosition = 0;

    public int forward = 1;
    public int reverse = -1;
    double programStartOrientation;
    public double stayOnHeading = 84.17;

    boolean hangSlidesDown = false;


    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "Ad2+xnL/////AAABmfD+XRaqfERPmvnHPZzg8b5xA35SCU5QWgaygrkAKRjWp+n" +
            "searSU8Zriv5xsNvOm3cLWfUa7gXGF1h09LWDH6+0QrZ6WVl11ygsh5wTa8IyIZGaPqHG9FjsccPCzNtSPpLZj3vpS4K797weILM" +
            "vElMa4xrSb/xSyn5zWwGEg5H931imaB8yFDkV7LIAxRJgfORqJcrOQ4WVjr6GxEVj2mjNkHNCKF57C1yyY8CYit5BcgDAkz4bosZ" +
            "0jPpvwCks1+trrm5kP+NIj6y49SD+NZh85IUiEITB9ebw49pvA9M8fki18jLYDIexUZ7fnCFj8oBGGnc0CCispwE2ST7ddUDo4" +
            "GmrSSkNLfUrDMjapPpK\n";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

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
        TeamMarker = hardwareMap.crservo.get("TeamMarker");
        IntakeFlapLeft = hardwareMap.servo.get("IntakeFlapLeft");


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
            //telemetry.addData("IMU", imuTurning.getHeading());
            telemetry.addData("FrontLeft Encoder", LeftTop.getCurrentPosition());
            telemetry.addData("If values are good, then run", 1);
            telemetry.addData("If I don't put the not !waitforstart or whatever here then it'll probably crash", 1);
            telemetry.update();
        }
        runtime.reset();
    }


    //Start TensorFlow functions (not movement based on TF)
    private void initVuforia() {
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
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
    //End TensorFlow functions (not movement based on TF)

    // Start General movement functions
    public void encoderDrive(double speed, double Inches, int direction, double heading, double timeout, boolean armUp) {
        double startTime = runtime.seconds();
        double Heading = 0;
        boolean notAtTarget = true;
        //if we give a very very specific value for our heading, than we stay on our current path
        //otherwise, we get use the gyroDrive to correct to our desired heading
        if (heading == 84.17){
            Heading = getHeading();
        }else{
            Heading = heading;
        }
        double target;
        if (opModeIsActive() ) {
            //make sure that the encoder on the front left motor (which, stupidly, is the only motor
            //we use for distance in this function) is reset to 0
            LeftTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LeftTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LeftBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RightTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RightBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //find the amount of encoder ticks to travel based off of the Inches var
            target = LeftTop.getCurrentPosition() + (int) (Inches * CountsPerInch * direction);
            //while the opmode is still running, and we're not at our target yet, and we haven't timed out
            while(opModeIsActive() && notAtTarget && Math.abs(target) - Math.abs(LeftTop.getCurrentPosition()) > 25
                    && (startTime + timeout > runtime.seconds())) {
                //use gyrodrive to set power to the motors.  We have the Heading Var decied earlier,
                // and speed and direction change base off of speed and direciton given by the user
                gyroDrive(Heading, speed, direction);
                if(armUp) {
                    unextendHangSlide(true);
                }else{
                    unextendHangSlide(false);
                }
            }
            ArmTop.setPower(0);
            ArmBottom.setPower(0);
            stopDriveMotors();
        }
    }
    public void encoderDriveSmooth(double speed, double Inches, int direction, double heading, double timeout) {
        double startTime = runtime.seconds();
        double Heading = 0;
        boolean notAtTarget = true;
        unextendHangSlide(true);
        //if we give a very very specific value for our heading, than we stay on our current path
        //otherwise, we get use the gyroDrive to correct to our desired heading
        if (heading == 84.17){
            Heading = getHeading();
        }else{
            Heading = heading;
        }
        double target;
        if (opModeIsActive() ) {
            //make sure that the encoder on the front left motor (which, stupidly, is the only motor
            //we use for distance in this function) is reset to 0
            LeftTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LeftTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LeftBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RightTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RightBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //find the amount of encoder ticks to travel based off of the Inches var
            target = LeftTop.getCurrentPosition() + (int) (Inches * CountsPerInch * direction);
            //while the opmode is still running, and we're not at our target yet, and we haven't timed out
            while(opModeIsActive() && notAtTarget && Math.abs(target) - Math.abs(LeftTop.getCurrentPosition()) > 25
                    && (startTime + timeout > runtime.seconds())) {
                //use gyrodrive to set power to the motors.  We have the Heading Var decied earlier,
                // and speed and direction change base off of speed and direciton given by the user
                gyroDrive(Heading, speed, direction);
            }
        }
    }
    public void gyroTurn(double speed, double angle) {
        LeftTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        double leftSpeed;//var to store left side of drivetrain speed
        double rightSpeed;//var to store right side of drivetrain speed
        double PosOrNeg; //Var for us to store which way we'll be turning.
        double minTurnSpeed = .4;//Minimum speed (and constant added to error) we will be turning
        double maxTurnSpeed = .7;//Maximum speed we'll be turning, we don't want to have a super high val
        // for this otherwise we won't decelerate for our target in time
        double error = getError(-angle);//set to a lower number so that if we are starting with a 0 heading it'll
        double turningSpeedError = 0;
        double desiredRotSpeed = .35;
        double Kp = .01;
        double Kd = .215;
        int errorCount = 0;
        int toleranceDegrees = 2;
        //still run through the loop, rather than
        double timer = runtime.seconds() + 1.25;
        while(errorCount < 5 && opModeIsActive() && timer > runtime.seconds()){
            unextendHangSlide(true);//make sure the hanging slide is going (or is) down

            //While we aren't within a degree of our target and the opmode is running
            //if the error has been less than the desired tolerance for 5 loops, terminate.  This keeps
            // the program from stopping and overshooting if we just happen onto a value.

            if(Math.abs(error) < 2){
                errorCount ++;
            }else{
                errorCount = 0;
            }
            error = getError(-angle);//get the difference between our current heading and our desired heading.
            turningSpeedError = IMU.getAngularVelocity().zRotationRate - desiredRotSpeed;
            double turningSpeed =minTurnSpeed + (Math.abs(error)*Kp)-(Math.sqrt(Math.abs((turningSpeedError)))-desiredRotSpeed)*Kd;

            /*f(Math.abs(IMU.getAngularVelocity().zRotationRate) < .2 && minTurnSpeed < .5 ){
                //get the angularVelocity (change in heading/time, basically) of the IMU, so we can see
                //we're spinning too slow to turn, so up the minimum power so we can turn again
                minTurnSpeed += .03;
            }else if(minTurnSpeed > maxTurnSpeed || Math.abs(IMU.getAngularVelocity().zRotationRate) > .8){
                //makes sure we don't have a runnaway minPower.
                minTurnSpeed = .3;
            }*/


            // This is the main part of the Proportional GyroTurn.  This takes a set power variable,
            // and adds the absolute value of error/200.  This allows for the robot to turn faster when farther
            // away from the target, and turn slower when closer to the target.  This allows for quicker, as well
            // as more accurate turning when using the GyroSensor
            PosOrNeg = Range.clip((int)error, -1, 1);
            //PosOrNeg lets us turn the correct direction, since otherwise we would have to always turn clockwise
            //leftSpeed  = Range.clip(minTurnSpeed + Math.abs(error)/150 , minTurnSpeed, maxTurnSpeed)* PosOrNeg;
            leftSpeed  = Range.clip(turningSpeed, .35, 1)* PosOrNeg;

            rightSpeed = -leftSpeed;//right side goes negative left, so we actually turn.

            // Set powers to motors
            LeftTop.setPower(leftSpeed);
            LeftBottom.setPower(leftSpeed);
            RightTop.setPower(rightSpeed);
            RightBottom.setPower(rightSpeed);

            //Show debug info in telemetry.
            telemetry.addData("Target", "%5.2f", angle);
            telemetry.addData("Speeds, Left,Right.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
            telemetry.addData("turning speed.",  IMU.getAngularVelocity().zRotationRate);
            telemetry.update();
        }
        stopDriveMotors();
        ArmBottom.setPower(0);
        ArmTop.setPower(0);

    }
    public double getError(double targetAngle) {
        //This function compares the current heading to the target heading, and returns the error
        double robotError;
        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getHeading();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    public void gyroDrive(double targetAngle, double targetSpeed, int direction) {
        //For use with other functions, but lets us use the gyro to keep the robot on a certain heading
        // it's proportional, so if for instance, a robot hits us, this will account for that, and
        // correct the robot's heading.  It's not smart enough to oversteer to make sure we're on the exact
        // same plain, but it's good enough for our use case since people shouldn't hit us in auton
        LeftTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double LeftPower = 0;
        double RightPower = 0;
        int Direction = -direction;
        double diff = -getError(targetAngle);
        // Play with this value for different sensitivities depending on different speeds
        double PVal = 15/targetSpeed;
        if(Direction == -1){
            //if we're traveling backwards, we want to add the difference to the opposite as we
            // would if we traveled forward
            //We're getting the targetSpeed, and adding the (difference/PVal)
            //The PVal is decided by dividing 15 (which is an arbitrary value determined by robot)
            //  by the target speed.  It was played around with, and decided on after testing.
            // By including a second method of tuning our speed changes, we can have a more,
            // or less, sensitive proportional drive depending not just on the error of our heading,
            // but depending on our target speed as well.  This means when we're traveling fast,
            // we change our values more, because it's actually a smaller percentage of it's overall
            // speed.  In contrast, while driving slowly, we make smaller speed changes.
            LeftPower = Direction*(targetSpeed+diff/PVal);
            RightPower = Direction*(targetSpeed-diff/PVal);
        }else{
            //same as above, but opposite
            LeftPower = Direction*(targetSpeed-diff/PVal);
            RightPower = Direction*(targetSpeed+diff/PVal);
        }
        //Make sure the powers are between 1 and -1.  This doesn't do much, other than ensure
        // stability of the code, and making sure it doesn't crash for a weird reason
        LeftTop.setPower(Range.clip(LeftPower, -1, 1));
        LeftBottom.setPower(Range.clip(LeftPower, -1, 1));
        RightTop.setPower(Range.clip(RightPower, -1, 1));
        RightBottom.setPower(Range.clip(RightPower, -1, 1));
    }
    public double getHeading(){
        //returns the Z axis (which is what you want if the Rev module is flat), for ease of use
        Orientation angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
    public void stopDriveMotors(){
        //just makes it easier to stop the motors, instead of having to write it out all the time
        LeftTop.setPower(0);
        LeftBottom.setPower(0);
        RightTop.setPower(0);
        RightBottom.setPower(0);
    }
    //End General movement functions


    //Start Rover Ruckus specific movement and logic functions


    //Start Crater side functions
    public void craterSideSample(){
        IntakeFlapLeft.setPosition(intakeFlapLeftOpen);
        ElapsedTime elapsedTime = new ElapsedTime();
        //should come immediately after unlatching
        //for the most part this should be able to be copy/pasted to the depotSideSample, though a few changes
        //for the team marker may have to be made.
        //gyroTurn(turningSpeed, 0); Removed, we never need to re-align after dropping.
        encoderDrive(.35, 2, 1, stayOnHeading, 2, true);
        gyroTurn(turningSpeed, 15);
        double time = elapsedTime.seconds();
        while(goldPosition == 0 && elapsedTime.seconds() < time+2 && opModeIsActive()){
            //wait for 3 seconds to make sure TFOD has time to process the frames
            //Otherwise, we may get incorrect readings, since it may have just not seen a mineral in time
            getGoldPositionOneMineral();
            unextendHangSlide(true);
        }
        if(goldPosition == 0){
            //failsafe, so that if this doesn't detect the right two minerals at least we'll still place
            // the team marker and park
            goldPosition = 1;
            //Position 1 is the leftmost mineral
        }
        gyroTurn(turningSpeed, decideFirstSampleheading());

        if(goldPosition == 1){
            encoderDrive(.35, 4, reverse, 30, 2, true);
            putArmDown();
            setIntakePower(.7);
            encoderDrive(.25, 3, forward, 30, 2, false);
            encoderDrive(.25, 4, forward, 30, 2, true);

        }else if(goldPosition == 2){
            encoderDrive(.35, 4, reverse, 0, 2, true);
            putArmDown();
            setIntakePower(.7);
            encoderDrive(.25, 3, forward, 0, 2, false);
            encoderDrive(.25, 4, forward, 0, 2, true);

        }else{
            encoderDrive(.35, 4, reverse, -30, 2, true);
            putArmDown();
            setIntakePower(.7);
            encoderDrive(.25, 7, forward, -30, 2, false);
            encoderDrive(.25, 2, forward, -30, 2, true);
        }
        sleep(500);
        encoderDrive(.35, 3, forward, stayOnHeading, 2, true);
    }
    public void craterDoubleSample(){
        encoderDriveSmooth(.5, 8, forward, 160, 2);
        encoderDriveSmooth(.65, 24, forward, 135, 1.5);
        if(goldPosition == 3){
            encoderDriveSmooth(.65, 12, reverse, 130, 2);
            encoderDriveSmooth(.5, 8, reverse, 115, 1.5);
            encoderDriveSmooth(.5, 12, forward, 125, 1.5);
        }else{
            encoderDrive(.65, 12, reverse, -decideSecondSampleheading(), 2, true);
            encoderDrive(.75, 24, forward, 90, 2, true);
        }
        deployTeamMarker();
        encoderDriveSmooth(.75, 12, reverse, 145, 3);
        encoderDriveSmooth(.75, 48, reverse, 138, 3);
        TeamMarker.setPower(0);
    }

    public void driveFromCraterAfterSampleToNearDepot(int inches, long delayUntil){
        while(runtime.seconds() < delayUntil && opModeIsActive()){
            sleep(20);
            telemetry.addData("Waiting...", delayUntil - runtime.seconds());
            telemetry.update();
        }
       //make this sleep(runtime - wanted time ) or something
        encoderDrive(.5, 46, forward, 80, 2.5, true);
        gyroTurn(turningSpeed, -128);//turn to the left, facing the depot
        double currentDistance = 200;
        /*while(opModeIsActive() && currentDistance > 110){
            currentDistance = FrontDistance.getDistance();
            gyroDrive(130, .4, forward);
        }*/
        encoderDrive(.5, inches-8, forward, 128, 3, true);
        TeamMarker.setPower(1);
        encoderDrive(.5, 8, forward, -128, 3, true);
    }
    public void craterSidePark(){
        encoderDrive(.85, 52, reverse, 137, 3.5, true);
    }
    public void craterSideParkArmInCrater(){
        encoderDriveSmooth(.75, 16, reverse, stayOnHeading, 3);
        encoderDriveSmooth(.75, 6, reverse, 110, 2);
        encoderDriveSmooth(.75, 6, reverse, 120, 2);
        encoderDriveSmooth(.75, 18, reverse, 80, 2);
        gyroTurn(turningSpeed, 0);
    }
    //End Crater Side functions

    //Start Depot Side Functions
    public void depotSideSample(){
        ElapsedTime elapsedTime = new ElapsedTime();
//should come immediately after unlatching
        //for the most part this should be able to be copy/pasted to the depotSideSample, though a few changes
        //for the team marker may have to be made.
        gyroTurn(turningSpeed, 0);
        encoderDrive(.35, 2, 1, stayOnHeading, 2, true);
        gyroTurn(turningSpeed, 15);
        while(goldPosition == 0 && elapsedTime.seconds() < 3 && opModeIsActive()){
            getGoldPositionOneMineral();
            unextendHangSlide(true);
        }
        if(goldPosition == 0){
            //failsafe, so that if this doesn't detect the right two minerals at least we'll still place
            // the team marker and park
            goldPosition = 1;
            //Position 1 is the leftmost mineral
        }
        gyroTurn(turningSpeed, decideFirstSampleheading());
        if(goldPosition == 2){
            encoderDrive(.5, 36, forward, stayOnHeading, 2.5, true);
        }else if(goldPosition == 3){
            encoderDrive(.5, 28, forward, stayOnHeading, 2.5, true);
        }else{
            encoderDrive(.5, 28, forward, stayOnHeading, 2.5, true);
        }
    }
    public void depotSideDeployAndPark(){
        if(goldPosition == 1){
            encoderDriveSmooth(.5, 20, forward, -42, 3);
            deployTeamMarker();//At this point we'll be on the edge of the depot and about to place the marker
            encoderDriveSmooth(.5, 8, forward, -42, 3);
            encoderDriveSmooth(.75, 36, reverse, -47, 5);
        }else if(goldPosition == 2){
            deployTeamMarker();
            sleep(200);
            encoderDrive(.5, 24, reverse, stayOnHeading, 2.5, true);
            gyroTurn(turningSpeed, 90);//At this point we'll be facing the other alliances crater-ish
            encoderDriveSmooth(.5, 35, reverse, -90, 2);//just hit the wall
            encoderDriveSmooth(.5, 24, reverse, -55, 2);//just hit the wall

            /*encoderDrive(.5, 18, forward, stayOnHeading, 2, true);
            gyroTurn(turningSpeed, -45);//face the near non-alliance wall
            encoderDrive(.35, 36, forward, stayOnHeading, 4, true);//just hit the wall
            encoderDrive(.2, 3, reverse, stayOnHeading, 2, true);//back away from the wall for turning clearance
            gyroTurn(turningSpeed, 50);//turn towards the depot
            encoderDrive(.5, 36, reverse, stayOnHeading, 3, true);*/
        }else{
            gyroTurn(turningSpeed, -45);//face the near non-alliance wall
            encoderDriveSmooth(.5, 8, forward, stayOnHeading, 2);//just hit the wall
            encoderDriveSmooth(.35, 8, forward, 90, 2);//just hit the wall
            encoderDrive(.5, 20, forward, 45, 2, true);//just hit the wall
            encoderDrive(.35, 2, reverse, stayOnHeading, 4, true);//just hit the wall
            gyroTurn(turningSpeed, 45);
            encoderDrive(.35, 3, forward, -42, 4, true);//just hit the wall
            deployTeamMarker();//At this point we'll be on the edge of the depot and about to place the marker
            sleep(250);
            gyroTurn(turningSpeed, 49);
            encoderDrive(.75, 36, reverse, -47, 5, true);
        }
        //should stay at 24 inches, adjust other parts to get proper distance
        encoderDriveSmooth(.35, 24, reverse, -50, 30);
        stopDriveMotors();
    }
    //End Depot Side Functions

    //Start deciding mineral positiokn
    public int decideFirstSampleheading(){
        int heading;
        if(goldPosition == 1){
            telemetry.addData("On your left", "Marvel reference");
            heading = -35;
        }else if (goldPosition == 2){
            telemetry.addData("Center", "Like Shaq");
            heading = 0;
        }else if(goldPosition == 3){
            telemetry.addData("Right", "Like I always am");
            heading = 35;
        }else{
            telemetry.addData("Something is very wrong", "Decide first sample heading function");
            //if this ever shows up, it's most likely that we didn't see the samples in @craterSideSample or something
            heading = 0;
        }
        telemetry.update();
        return heading;
    }
    public double decideSecondSampleheading(){
        double heading = getHeading();
        if(goldPosition == 1){
            telemetry.addData("On your left", "Marvel reference");
            heading = -70   ;
        }else if (goldPosition == 2){
            telemetry.addData("Center", "Like Shaq");
            heading = -100;
        }else if(goldPosition == 3){
            telemetry.addData("Right", "Like I always am");
            heading = -130;
        }else{
            telemetry.addData("Something is very wrong", "Decide first sample heading function");
            //if this ever shows up, it's most likely that we didn't see the samples in @craterSideSample or something
            heading = 0;
        }
        telemetry.update();
        return heading;
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
    //End deciding mineral position

    //Start hanging system code
    public void unlatch(int inchesToUnlatch){
        HangingSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        HangingSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        HangingSlide.setTargetPosition((int) ticksPerHangingInch * -inchesToUnlatch);
        HangCamLeft.setPosition(hangCamLeftUnengagedPos);
        HangCamRight.setPosition(hangCamRightUnengagedPos);
        double timer = runtime.seconds() + 1;
        while(timer > runtime.seconds() && opModeIsActive()){
            keepMineralArmUp();
        }
        double newTimer = runtime.seconds() + 6;

        while(HangingSlide.isBusy() && newTimer > runtime.seconds() && opModeIsActive()){
            HangingSlide.setPower(1);
            keepMineralArmUp();
        }
        HangingSlide.setPower(0);
        ArmTop.setPower(0);
        ArmBottom.setPower(0);
    }
    public void unextendHangSlide(boolean keepArmUp){
        //this is made so it can be in a loop by itself, or in another loop.
        HangingSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        HangingSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if(!hangSlidesDown) {
            if (hangSlideIsExtended()) {
                HangingSlide.setPower(.5);
            } else {
                HangingSlide.setPower(0);
                hangSlidesDown = true;
            }
        }
        if (keepArmUp) {
            keepMineralArmUp();
        }
    }
    public boolean hangSlideIsExtended(){
        if(HangSlideLimit.getState() == false){
            return false;
        }else {
            return true;
        }
    }
    //End hanging system code

    //Start mineral system code
    public void rotateArm(int desiredRot) {
        while(opModeIsActive() && !armIsDown()) {
            double armRotError = (Math.abs(potRotation())-Math.abs(desiredRot));
            armPower = Range.clip(armRotError*armPVal, -.5, .5);
            ArmTop.setPower(armPower);
            ArmBottom.setPower(armPower);
        }
        ArmTop.setPower(0);
        ArmBottom.setPower(0);
    }
    public void putArmDown() {
        while(opModeIsActive() && !armIsDown()) {
            putMineralArmDown();
            setIntakePower(.8);
        }
        ArmTop.setPower(0);
        ArmBottom.setPower(0);
    }
    public boolean armIsDown() {
        if (potRotation() < armDownRotation - 20) {
            return false;
        } else {
            return true;
        }
    }
    public void keepMineralArmUp(){
        double armRotError = (Math.abs(potRotation())-Math.abs(armScoringRotation));

        armPower = Range.clip(armRotError*armPVal, -1, 1);
        ArmTop.setPower(armPower);
        ArmBottom.setPower(armPower);
    }
    public void putMineralArmDown(){
        double armRotError = (Math.abs(potRotation())-Math.abs(armDownRotation));

        armPower = Range.clip(armRotError*armPVal, -.5, .5);
        ArmTop.setPower(armPower);
        ArmBottom.setPower(armPower);
    }
    public double potRotation(){
        double potRotation = ArmPot.getVoltage()/potMagicNumber;
        return potRotation;
    }
    public void extendMineralArm(int inchesToExtend){
        ArmSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmSlide.setTargetPosition((int) ticksToExtendMineralArmInch * inchesToExtend);
        double timer = runtime.seconds() + 1;
        while(timer > runtime.seconds() && opModeIsActive()){
            keepMineralArmUp();
        }
        while(ArmSlide.isBusy() && opModeIsActive()){
            keepMineralArmUp();
            ArmSlide.setPower(1);
        }
        ArmSlide.setPower(0);
    }
    public void setIntakePower(double power){
        Range.clip(power, -.7, .7);//393s have a power limit of .7.  Higher and they won't spin
        IntakeLeft.setPower(power);
        IntakeRight.setPower(power);
        IntakeFlapLeft.setPosition(intakeFlapLeftOpen);
    }
    //End mineral system code

    //Start miscellaneous system code
    public void deployTeamMarker(){
        TeamMarker.setPower(1);
    }
    //End miscellaneous system code

    //End of autonomous housekeeping
    public void endAuto(boolean endWithArmUp){
        encoderDrive(.35, 3, reverse, stayOnHeading, 1, true);
        ArmBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ArmTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        boolean putArmDown = true;
        tfod.deactivate();
        //telemetry for autonomous testing to see any factors that may have went wrong
        TeamMarker.setPower(0);
        if(endWithArmUp) {
            while (opModeIsActive() && hangSlideIsExtended()) {
                unextendHangSlide(true);
            }
        }else{
            //ending with arm parked in crater
            extendMineralArm(14);
            while (opModeIsActive()) {
                if(potRotation() < 130) {
                    putMineralArmDown();
                }else{
                    ArmTop.setPower(0);
                    ArmBottom.setPower(0);                }
                if(hangSlideIsExtended()){
                    unextendHangSlide(false);
                }else{
                    HangingSlide.setPower(0);
                }
            }
        }
        telemetry.addData("No Glyphs", "Cuz that was last year");
        telemetry.update();
    }
}