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


@TeleOp(name="Teleop Test", group="TEST")
public class Teleop3 extends OpMode {
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
    public Servo HangCamLeft = null;
    public Servo HangCamRight = null;
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

    double armScoringRotation = 40;
    double armkP = .015;//Change this for faster or slower auto arm rotation, .2 optimal?
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
        ArmTop.setPower(gamepad1.left_stick_y);
        ArmBottom.setPower(gamepad1.right_stick_y);
        doTelemetry();
    }
    public void doTelemetry(){
        telemetry.addData("Pot Voltage", ArmPot.getVoltage());
        telemetry.addData("Pot Position", (ArmPot.getVoltage()/potMagicNumber));
        telemetry.addData("left Joy,", gamepad1.left_stick_y);
        telemetry.addData("Right Joy,", gamepad1.right_stick_y);
        telemetry.addData("left power,", leftPower);
        telemetry.addData("right power,", rightPower);
        telemetry.addData("hanging power,", hangingMotorPower);
        telemetry.addData("arm power,", armPower);
        telemetry.addData("Magnetic switch", HangSlideLimit.getState());
        telemetry.update();
    }
}