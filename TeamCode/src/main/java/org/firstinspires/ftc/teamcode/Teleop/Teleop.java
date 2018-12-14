package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.util.Range.clip;


@TeleOp(name="Teleop", group="TELEOP")
public class Teleop extends OpMode {
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
    public Servo HangCamLeft;
    public Servo HangCamRight;
    public Servo TeamMarker;
    public Servo IntakeFlapLeft;
    public Servo IntakeFlapRight;

    public DigitalChannel HangSlideLimit;
    public AnalogInput ArmPot;
    double potMagicNumber = .01222;
    double potRotation;

    // Declare Variables
    double leftPower = 0;
    double rightPower = 0;

    double armPower = 0;
    double armSlidePower = 0;

    double hangingMotorPower = 0;
    double intakePower = 0;
    double armRotatePower = 0;

    double teamMarkerDeploy = -.1;
    double teamMarkerResting = .3;

    boolean hangRatchetEngaged = true;
    double hangCamLeftEngagedPos = 1;
    double hangCamLeftUnengagedPos = 0;
    double hangCamRightEngagedPos = 0;
    double hangCamRightUnengagedPos = 1;
    double intakeFlapLeftOpen = 0;
    double intakeFlapLeftClosed = 1;
    double intakeFlapRightOpen = 1;
    double intakeFlapRightClosed = 0;

    double armScoringRotation = 65;
    double armPVal = .015;//Change this for faster or slower auto arm rotation, .2 optimal?
    double armRotError = 0;


    double hangSlideDownPos = 0;


    public ElapsedTime runtime = new ElapsedTime();

    public void init() {
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
        IntakeFlapLeft = hardwareMap.servo.get("IntakeFlapLeft");
        IntakeFlapRight = hardwareMap.servo.get("IntakeFlapRight");

        HangCamLeft = hardwareMap.servo.get("HangCamLeft");
        HangCamRight = hardwareMap.servo.get("HangCamRight");

        TeamMarker = hardwareMap.servo.get("TeamMarker");

        HangSlideLimit = hardwareMap.get(DigitalChannel.class, "HangSlideLimit");
        HangSlideLimit.setMode(DigitalChannel.Mode.INPUT);

        ArmPot = hardwareMap.analogInput.get("ArmPot");

        LeftTop.setDirection(DcMotorSimple.Direction.FORWARD);
        LeftBottom.setDirection(DcMotorSimple.Direction.REVERSE);
        RightTop.setDirection(DcMotorSimple.Direction.REVERSE);
        RightBottom.setDirection(DcMotorSimple.Direction.FORWARD);

        HangingSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeRight.setDirection(DcMotorSimple.Direction.REVERSE);


        LeftTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ArmTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        HangingSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

    @Override
    public void init_loop() {
        //There's the small problem of the E4s crashing, which is fixed by sending telemetry to
        //the phone during initialization()
        telemetry.addData("Status", "Waiting in Init plz don't crash E4s");
        telemetry.addData("Cuz Like", "That would really suck");
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        potRotation = ArmPot.getVoltage()/potMagicNumber;


        if(gamepad2.right_bumper || gamepad1.dpad_up){
            //This section of code is a proportional system that rotates our mineral scoring system
            //into place.  This lets Samuel rotate the arm into place with just the push of a button
            // instead of trying to rotate it into place manually.
            armRotError = (Math.abs(potRotation)-Math.abs(armScoringRotation));
            armPower = Range.clip(armRotError*armPVal, -1, 1);
            //This bottom line multiplies the Pvalue (.015) by the erorr, to give us motor power
            //this means that further away from the proper rotation (when the arm is down) the power
            //is greater than when it's close to the target.
        }else {
            //All these loops do is keep our arm from breaking itself.  We use the Rev potentiometer
            // to get the rotation of the arm, and have done testing with these limits to make sure
            // they are in the right place.
            if (potRotation <= 35) {
                //if the arm is too far back, move it up slowly
                armPower = Range.clip(-gamepad2.left_stick_y - .2, -1, 0);
            } else if (potRotation < 40) {
                //if it's at the limits, allow no more movement backward
                armPower = Range.clip(-gamepad2.left_stick_y, -1, 0);
            } else if (potRotation >= 40 && potRotation <= 50) {
                //if it's close to the limits, let it move slowly
                armPower = Range.clip(-gamepad2.left_stick_y, -1, .25);
            } else if (potRotation >= 175 && potRotation <= 195) {
                //if it's close to the limits, let it move slowly
                armPower = Range.clip(-gamepad2.left_stick_y, -.35, 1);
            } else if (potRotation >= 195 && potRotation <= 200) {
                //if it's at the down limits, allow no more forward rotation
                armPower = Range.clip(-gamepad2.left_stick_y, 0, 1);
            } else if (potRotation >= 200) {
                //if it's past the limits, move it back into it's limits slowly
                armPower = Range.clip(-gamepad2.left_stick_y + .2, 0, 1);
            } else {
                //Otherwise, if it's not being controlled by limits, just give full power to the joystick
                armPower = -gamepad2.left_stick_y;
            }
        }

        if(gamepad2.left_trigger > .1){
            //This is a slow mode for the
            armPower = armPower * .35;
        }


        if(gamepad2.right_trigger > .1) {
            //Slow intake mode for Samuel
            intakePower = Range.clip(gamepad2.right_trigger/3,-.7,.7);
        }else if(gamepad1.right_bumper) {
            //Let Isaac set full power to the intake
            intakePower = .7;
        }else if(gamepad1.left_bumper || gamepad2.start) {
            //Reverse the intake
            intakePower = -.7;
        }else{
            //If none of the other things are happening, intake is off
            intakePower = 0;
        }

        if(gamepad2.dpad_down){
            //Silver side, automatically open and spin intake at .15 speed
            IntakeFlapLeft.setPosition(intakeFlapLeftOpen);
            IntakeFlapRight.setPosition(intakeFlapRightOpen);
            intakePower = .25;
        }else if(gamepad2.dpad_up) {
            //button to close the intake
            IntakeFlapLeft.setPosition(intakeFlapLeftClosed);
            IntakeFlapRight.setPosition(intakeFlapRightClosed);
        }else if(gamepad2.dpad_left){
            //Gold side, automatically open halfway and spin intake at full speed
            IntakeFlapLeft.setPosition(.4);
            IntakeFlapRight.setPosition(.6);
            intakePower = .7;
        }else{
            //If we're not scoring, then close the intake
            IntakeFlapLeft.setPosition(intakeFlapLeftClosed);
            IntakeFlapRight.setPosition(intakeFlapRightClosed);
        }

        /*if(gamepad1.right_trigger > .1){
            if(hangRatchetEngaged){

                //wait a second before allowing power
                hangRatchetEngaged= false;
            }else{
                //motor power = gamepad1.right_trigger;
            }
        }
        if(gamepad1.left_trigger > .1){
            hangRatchetEngaged = true;
            //motorPower = trigger down
        }*/


        if(gamepad2.left_bumper){
            if (HangSlideLimit.getState() == false) {
                //hanging slide is down, sensed by the limit switch
                hangingMotorPower = Range.clip(gamepad2.right_stick_y, -1, 0);
            } else {
                hangingMotorPower = gamepad2.right_stick_y;
            }
        }else {
            if (HangSlideLimit.getState() == false) {
                //hanging slide is down
                hangingMotorPower = Range.clip(-gamepad1.left_trigger, -1, 0);
            } else {
                if(gamepad1.right_trigger > .1){
                //Allow control on gamepad 1 to hang, which should help increase speed of hanging
                    hangingMotorPower = gamepad1.right_trigger;
                }else if(gamepad1.left_trigger > .1){
                    hangingMotorPower = -gamepad1.left_trigger;
                }else{
                    hangingMotorPower = 0;
                }
            }
            armSlidePower = -gamepad2.right_stick_y;
        }

        if(gamepad1.a){
            //If either of the a buttons are pressed, engage the ratchet system
            hangRatchetEngaged = true;
        }else if (gamepad1.b){
            //if either of the b buttons are pressed, disengage the ratchet system
            hangRatchetEngaged = false;
        }


        //Set drivetrain motor power
        leftPower = gamepad1.left_stick_y;
        rightPower = gamepad1.right_stick_y;

        //set the power variables to the motors
        setPowers();

        //print out telemetry to the driver station
        doTelemetry();
    }

    public void setPowers(){
        //Set motor powers
        LeftTop.setPower(leftPower);
        LeftBottom.setPower(leftPower);
        RightTop.setPower(rightPower);
        RightBottom.setPower(rightPower);

        ArmTop.setPower(armPower);
        ArmBottom.setPower(armPower);
        ArmSlide.setPower(armSlidePower);

        HangingSlide.setPower(hangingMotorPower);


        //Set servo positions/powers

        //set intake speed
        IntakeLeft.setPower(intakePower);
        IntakeRight.setPower(intakePower);

        //Set ratchet servo positions
        if(hangRatchetEngaged) {
            HangCamLeft.setPosition(hangCamLeftEngagedPos);
            HangCamRight.setPosition(hangCamRightEngagedPos);
        }else{
            HangCamLeft.setPosition(hangCamLeftUnengagedPos);
            HangCamRight.setPosition(hangCamRightUnengagedPos);
        }


    }
    public void doTelemetry(){
        telemetry.addData("Pot Voltage", ArmPot.getVoltage());
        telemetry.addData("Pot Position", (ArmPot.getVoltage()/potMagicNumber));
        telemetry.addData("left power,", leftPower);
        telemetry.addData("right power,", rightPower);
        telemetry.addData("hanging power,", hangingMotorPower);
        telemetry.addData("arm power,", armPower);
        telemetry.addData("Magnetic switch", HangSlideLimit.getState());
        telemetry.update();
    }
}