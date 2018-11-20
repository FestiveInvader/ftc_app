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

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;

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

    boolean hangRatchetEngaged = true;
    double hangCamLeftEngagedPos = 1;
    double hangCamLeftUnengagedPos = 0;
    double hangCamRightEngagedPos = 0;
    double hangCamRightUnengagedPos = 1;

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
        ArmTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ArmSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IntakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        /*ArmTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmMid.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
*/

    }

    @Override
    public void init_loop() {
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

        if(potRotation <= 45) {
            armPower = Range.clip(gamepad2.left_stick_y-.2, -1, 0);
        }else if(potRotation < 50) {
            armPower = Range.clip(gamepad2.left_stick_y, -1, 0);
        }else if(potRotation >= 50 && potRotation <= 65){
            armPower = Range.clip(gamepad2.left_stick_y, -1, .25);
        }else if (potRotation >= 175 && potRotation <= 195){
            armPower = Range.clip(gamepad2.left_stick_y, -.35, 1);
        }else if (potRotation >= 195 && potRotation <=200){
            armPower = Range.clip(gamepad2.left_stick_y, 0, 1);
        }else if (potRotation >= 200){
            armPower = Range.clip(gamepad2.left_stick_y+.2, 0, 1);
        }else{
            armPower = gamepad2.left_stick_y;
        }

        if(gamepad2.left_trigger > .1){
            armPower = armPower * gamepad2.left_trigger;
        }

        if(HangSlideLimit.getState() == false){
            //hanging slide is down
            hangingMotorPower = Range.clip(gamepad2.right_stick_y, -1, 0);
        }else {
            //if(HangingSlide.getCurrentPosition() < ticksToExtendHang)
            hangingMotorPower = gamepad2.right_stick_y;
        }
        if(gamepad2.a){
            hangRatchetEngaged = true;
        }else if (gamepad2.b){
            hangRatchetEngaged = false;
        }


        if(gamepad1.right_trigger > .1){
            armSlidePower = gamepad1.right_trigger;
        }else if(gamepad1.left_trigger > .1){
            armSlidePower = gamepad1.left_trigger;
        }else{
            armSlidePower = 0;
        }

        if(gamepad1.right_bumper) {
            intakePower = .7;
        }else{
            intakePower = 0;
        }

        leftPower = gamepad1.left_stick_y;
        rightPower = gamepad1.right_stick_y;

        setPowers();
        doTelemetry();
    }

    public void setPowers(){
        LeftTop.setPower(leftPower);
        LeftBottom.setPower(leftPower);
        RightTop.setPower(rightPower);
        RightBottom.setPower(rightPower);

        ArmTop.setPower(armPower);
        ArmBottom.setPower(armPower);
        ArmSlide.setPower(armSlidePower);

        HangingSlide.setPower(hangingMotorPower);



        IntakeLeft.setPower(intakePower);
        IntakeRight.setPower(intakePower);

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
        telemetry.addData("Magnetic switch", HangSlideLimit.getState());
        telemetry.update();
    }
}