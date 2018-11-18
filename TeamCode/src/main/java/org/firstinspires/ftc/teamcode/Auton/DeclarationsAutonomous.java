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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

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

    //public DcMotor ArmTop = null;
    //public DcMotor ArmMid = null;
    //public DcMotor ArmBottom = null;
    //public DcMotor ArmSlide = null;


    // Declare Servos
    public CRServo IntakeLeft;
    public CRServo IntakeRight;
    public Servo HangCamLeft;
    public Servo HangCamRight;
    //public CRServo IntakeLeft;                  // Rev SRS
    //public Servo PTOShifterLeft;                      // Rev SRS
    //public Servo PTOShifterRight;
    public DigitalChannel HangSlideLimit;
    public AnalogInput ArmPot;
    public BNO055IMU IMU;

    // Variables used  in functions
    double CountsPerRev = 537.6;    // Andymark NeveRest 20 encoder counts per revolution
    double GearRatio = 56/42;
    double WheelDiameterInches = 3.0;     // For figuring circumference
    double CountsPerInch = ((CountsPerRev / ((WheelDiameterInches * 3.1415))/GearRatio));

    double rev40TicksPerRev = 1120;
    double hangingPulleyDiameter = 1.1;
    double hangingGearRatio = 60/40;
    double ticksPerHangingRev = rev40TicksPerRev*hangingGearRatio;
    double ticksPerHangingInch =  ticksPerHangingRev / ((hangingPulleyDiameter * 3.1415)/hangingGearRatio);

    double HEADING_THRESHOLD = 2;      // As tight as we can make it with an integer gyro
    double P_TURN_COEFF = .2;     // Larger is more responsive, but also less stable
    double P_DRIVE_COEFF = .15;     // Larger is more responsive, but also less stable
    double turningSpeed = .175;

    double potMagicNumber = .01222;

    boolean hangRatchetEngaged = true;
    double hangCamLeftEngagedPos = 1;
    double hangCamLeftUnengagedPos = 0;
    double hangCamRightEngagedPos = 0;
    double hangCamRightUnengagedPos = 1;

    //empty is 0, grey is 1, brown is 2,

    int forward = 1;
    int reverse = -1;
    double programStartOrientation;
    double stayOnHeading = 84.17;



    VuforiaLocalizer vuforia;
    //VuforiaHardware vuforiaHardware;
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

        HangSlideLimit = hardwareMap.get(DigitalChannel.class, "HangSlideLimit");
        HangSlideLimit.setMode(DigitalChannel.Mode.INPUT);

        ArmPot = hardwareMap.analogInput.get("ArmPot");

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

        ElapsedTime timer = new ElapsedTime();
        while(!isStarted()) {
            telemetry.addData("IMU", getHeading());
            telemetry.addData("FrontLeft Encoder", LeftTop.getCurrentPosition());
            telemetry.addData("If values are good, then run", 1);
            telemetry.addData("If I don't put this here then it'll probably crash", 1);
            telemetry.update();
        }
    }

    // Start General movement functions
    public void drive(double speed, int direction, double time){
        double startingHeading = getHeading();
        double timeStarted = runtime.time();
        LeftTop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightTop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //while running, and haven't timed-out, and
        // while runtime < 29.75 (to make sure we don't keep going after auton ends)
        while(opModeIsActive() && runtime.time() - timeStarted < time && runtime.seconds() < 29.75) {
            //use gyrodrive, so we can push stuff but stay on heading
            gyroDrive(startingHeading, speed, direction);
        }
        stopDriveMotors();
    }

    public void EncoderDrive(double speed, double Inches, int direction, double heading, double timeout) {
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
            }
            stopDriveMotors();
        }
    }
   /* public void EncoderDriveAccelDecel(double speed, double inches, double decelInches, int direction, double heading, double timeout){
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
            FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double power = 0;
            double error;

            target = FrontLeft.getCurrentPosition() + (int) (inches * CountsPerInch * direction);
            double decelTicks = (int) (decelInches * CountsPerInch);
            //while the absolute value of the target minus the AV of the FL encoder value > 25 {
            //if(AV of FL ecnoders < decelTicks position) {drive normally}
            //else{set drive motor powers to .15, so that momentum from full speed means we decel}
            //It's dirty, and doesn't actually technically decel, but this was thrown together
            //the day before we left for Houston, so sue me
            while(Math.abs(target) - Math.abs(FrontLeft.getCurrentPosition()) > 25 && runtime.seconds() < 28.5 && (startTime + timeout > runtime.seconds())) {
                double motorPos = Math.abs(FrontLeft.getCurrentPosition());
                error = Math.abs(target) - Math.abs(motorPos);
                //decel
                if(Math.abs(motorPos) < Math.abs(target) - Math.abs(decelTicks)){
                    gyroDrive(Heading, speed, direction);
                    telemetry.addData("FrontLeftPwr", FrontLeft.getPower());
                    telemetry.addData("In speeeeeeeed", FrontLeft.getPower());
                    telemetry.update();
                }else {
                    gyroDrive(Heading, Range.clip(power, .15, 1), direction);
                    telemetry.addData("FrontLeftPwr", FrontLeft.getPower());
                    telemetry.addData("In Decel", FrontLeft.getPower());
                    telemetry.update();
                }

            }
            stopDriveMotors();
        }
    }
*/
    public void gyroTurn(double speed, double angle) {
        // keep looping while we are still active, and not on heading.
        //uses onHeading to actually turn the robot/figure out error
        while (opModeIsActive() && !onHeading(speed, -angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.addData("Target Rot", angle);
            telemetry.addData("Current Rot", getHeading());
            telemetry.update();
        }
    }
    boolean onHeading(double speed, double angle, double PCoeff) {
        //This function is a boolean, meaning it can be used in an if/while statement. For instance:
        //while(!onHeading){} would run all this code, until onHeading returns true
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;
        double PosOrNeg = 1;
        double SpeedError;
        double error = getError(angle);
        double minTurnSpeed = .15;
        double maxTurnSpeed = .5;
        // determine turn power based on +/- error
        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0;
            rightSpeed = 0;
            onTarget = true;
        }
        else {
            // This is the main part of the Proportional GyroTurn.  This takes a set power variable,
            // and adds the absolute value of error/150.  This allows for the robot to turn faster when farther
            // away from the target, and turn slower when closer to the target.  This allows for quicker, as well
            // as more accurate turning when using the GyroSensor
            PosOrNeg = Range.clip((int)error, -1, 1);
            steer = getSteer(error, PCoeff);
            leftSpeed  = Range.clip(speed + Math.abs(error/175) , minTurnSpeed, maxTurnSpeed)* PosOrNeg;

            rightSpeed = -leftSpeed;
        }

        // Set motor speeds.
        LeftTop.setPower(leftSpeed);
        LeftBottom.setPower(leftSpeed);
        RightTop.setPower(rightSpeed);
        RightBottom.setPower(rightSpeed);
        // Display debug info in telemetry.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Left,Right.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
        return onTarget;
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
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * (2*PCoeff), -1, 1);
    }
    public void gyroDrive(double targetAngle, double targetSpeed, int direction) {
        //For use with other functions, but lets us use the gyro to keep the robot on a certain heading
        // it's proportional, so if for instance, a robot hits us, this will account for that, and
        // correct the robot's heading.  It's not smart enough to oversteer to make sure we're on the exact
        // same plain, but it's good enough for our use case since people can't cross over in RR1
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
            //The PVal is decided by dividing 15 (which is an arbitrary value) by the target speed.
            // It was played around with, and decided on after testing.
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
    } //driveAdjust
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
   /* public void findWall(double speed, double distance, double Timeout){
        int badLoopTimer = 0;
        double startHeading = getHeading();
        boolean foundWall = false;
        double timeout = runtime.seconds() + Timeout;
        int ThisLoopDistance;
        while (opModeIsActive() && !foundWall && (runtime.seconds() < timeout)) {
            //we can tell if the sensor value is bad, but in this case we don't do anything with it
            ThisLoopDistance = BackDistance.getDistance();
            if(ThisLoopDistance > 200 || ThisLoopDistance < 21){
                //sensor val is bad, skip this loop
                moveBy(speed, 0, 0);
                badLoopTimer++;
            }else if(ThisLoopDistance > distance){
                moveBy(speed, 0, 0);
            }else{
                stopDriveMotors();
                foundWall = true;
            }
            telemetry.addData("Distance", BackDistance.getDistance());
            telemetry.update();
            smartIntake();
        }
    }
   */
    /*public void goToDistance(double targetSpeed, double distance,  I2CXLv2 distanceSensor, double timeout, int tolerance){
        double startTime = runtime.seconds();
        double maxTime = startTime + timeout;
        double startHeading = getHeading();
        boolean foundTarget = false;
        int ThisLoopDistance;
        while (opModeIsActive() && !foundTarget && maxTime - runtime.seconds() > .1) {
            ThisLoopDistance = BackDistance.getDistance();
            double error = distance - ThisLoopDistance;
            int Direction = (int) Range.clip(error, -1, 1);

            if(ThisLoopDistance > 500 || ThisLoopDistance < 21){
                gyroDrive(startHeading, Range.clip(Math.abs(error/70), .135, targetSpeed), Direction);
                //sensor val is bad, stop bot so it doesn't go too far
            }else if(ThisLoopDistance > distance + tolerance || ThisLoopDistance < distance - tolerance){
                gyroDrive(startHeading, Range.clip(Math.abs(error/70), .135, targetSpeed), Direction);
            }else{
                stopDriveMotors();
                foundTarget = true;
            }
            telemetry.addData("Distance", ThisLoopDistance);
            telemetry.addData("error", error);
            telemetry.addData("speed", FrontLeft.getPower());
            telemetry.update();
        }
        stopDriveMotors();
    }
    */
    //End General movement functions

    //Start Rover Ruckus specific movement and logic functions
    public void unlatch(){
        HangingSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int inchesToUnlatch = 26;
        HangingSlide.setTargetPosition((int) ticksPerHangingInch * inchesToUnlatch);
        HangCamLeft.setPosition(hangCamLeftUnengagedPos);
        HangCamRight.setPosition(hangCamRightUnengagedPos);
        HangingSlide.setPower(1);
        sleep(5000);
        EncoderDrive(.25, 6, forward, stayOnHeading, 5);
        HangCamLeft.setPosition(hangCamLeftEngagedPos);
        HangCamRight.setPosition(hangCamRightEngagedPos);
    }
    public void unextendHangSlide(){
        //this is made so it can be in a loop by itself, or in another loop.
        HangingSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(hangSlideIsExtended()){
            HangingSlide.setPower(-.5);
        }else{
            HangingSlide.setPower(0);
        }
    }
    public boolean hangSlideIsExtended(){
        if(HangSlideLimit.getState() == false){
            return false;
        }else {
            return true;
        }
    }
    public void endAuto(){
        //telemetry for autonomous testing to see any factors that may have went wrong
        telemetry.addData("No Glyphs", "Cuz that was last year");
        telemetry.update();
    }
        // End Regular 85 point functions

        // Start Multi-glyph functions

}