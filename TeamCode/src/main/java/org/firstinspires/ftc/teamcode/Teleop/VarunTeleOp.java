package org.firstinspires.ftc.teamcode.FTC_January;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "VarunTeleOp", group = "TeleOp")
public class VarunTeleOp extends OpMode {

    // Declare drive motors
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor rightFront;

    double leftBackPower;
    double rightBackPower;
    double leftFrontPower;
    double rightFrontPower;

    //Declare Servos
    private Servo linearslide;
    private Servo intake;
    private Servo dump;

    @Override
    public void init() {
        // Initialize drive motors
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        rightFront = hardwareMap.dcMotor.get("leftFront");
        leftFront = hardwareMap.dcMotor.get("rightFront");

        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        //Initialize Servo
        linearslide = hardwareMap.servo.get("linearslide");
        intake = hardwareMap.servo.get("intake");
        dump = hardwareMap.servo.get("dump");

    }

    @Override
    public void loop() {

        leftBackPower = gamepad1.left_stick_y;
        leftFrontPower = gamepad1.left_stick_y;
        rightBackPower = gamepad1.right_stick_y;
        rightFrontPower = gamepad1.right_stick_y;

        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        rightFront.setPower(rightFrontPower);
        leftFront.setPower(leftFrontPower);

        //Set Position to Forward 1
        if (gamepad1.y) {
            leftFront.setPower(1);
            rightBack.setPower(1);
            leftBack.setPower(1);
            rightFront.setPower(1);
        }

        //Set Position to Backward 1
        if (gamepad1.a) {
            leftFront.setPower(-1);
            rightBack.setPower(-1);
            leftBack.setPower(-1);
            rightFront.setPower(-1);
        }

        //Strafing to the right
        if (gamepad1.b) {
            leftBack.setPower(0.5);
            leftFront.setPower(-0.5);
            rightBack.setPower(-0.5);
            rightFront.setPower(0.5);
        }

        //Strafing to the left
        if (gamepad1.x) {
            leftBack.setPower(-0.5);
            leftFront.setPower(0.5);
            rightBack.setPower(0.5);
            rightFront.setPower(-0.5);
        }

        //Move Linear Slide Up
        if (gamepad2.dpad_up) {
            linearslide.setPosition(1);
        }

        //Move Linear Slide Down
        if (gamepad2.dpad_down) {
            linearslide.setPosition(0);
        }

        //Intake Mechanism Pickup
        if (gamepad2.x) {
            intake.setPosition(1);
        }

        //Dumping On Top Of Linear Slide
        if (gamepad2.y) {
            dump.setPosition(-1);

        }
    }
}





