package org.firstinspires.ftc.teamcode.FTC_January;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "VarunTeleOp", group = "TeleOp")
public class Teleop2 extends OpMode {

    // Declare drive motors
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor rightFront;

    double leftBackPower;
    double rightBackPower;
    double leftFrontPower;
    double rightFrontPower;

    double leftBackStrafe;
    double rightBackStrafe;
    double leftFrontStrafe;
    double rightFrontStrafe;

    //Declare Servos
    private Servo linearslide;
    private Servo intake;
    private Servo dump;
    private Servo arm;

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
        arm = hardwareMap.servo.get("arm");
        intake = hardwareMap.servo.get("intake");
        dump = hardwareMap.servo.get("dump");

    }

    @Override
    public void loop() {

        //Move Forward With Controllers
        leftBackPower = gamepad1.left_stick_y;
        leftFrontPower = gamepad1.left_stick_y;
        rightBackPower = gamepad1.right_stick_y;
        rightFrontPower = gamepad1.right_stick_y;

        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        rightFront.setPower(rightFrontPower);
        leftFront.setPower(leftFrontPower);

        //Strafe Using Controllers
        rightBackStrafe = gamepad1.right_stick_x;
        leftBackStrafe = gamepad1.left_stick_x;
        leftFrontStrafe = gamepad1.left_stick_x;
        rightFrontStrafe = gamepad1.right_stick_x;

        leftBack.setPower(leftBackStrafe);
        rightBack.setPower(rightBackStrafe);
        rightFront.setPower(rightFrontStrafe);
        leftFront.setPower(leftFrontStrafe);

        //To Move Forward
        if (gamepad1.left_stick_y>0) {
            leftBack.setPower(1);
            rightBack.setPower(1);
            leftBack.setPower(1);
            rightFront.setPower(1);
        }

        //To Move Backward
        if (gamepad1.left_stick_y<0) {
            leftBack.setPower(0);
            rightBack.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
        }

        //To strafe to the right
        if (gamepad1.left_stick_x<0) {
            leftBack.setPower(0.5);
            leftFront.setPower(-0.5);
            rightBack.setPower(-0.5);
            rightFront.setPower(0.5);
        }

        //To strafe to the left
        if (gamepad1.left_stick_x>0) {
            leftBack.setPower(-0.5);
            leftFront.setPower(0.5);
            rightBack.setPower(0.5);
            rightFront.setPower(-0.5);
        }

        if(gamepad1.y){
            arm.setPosition(1);
            arm.setDirection(Servo.Direction.FORWARD);
        }

        if(gamepad1.a){
            arm.setPosition(0);
            arm.setDirection(Servo.Direction.REVERSE);
        }

        if(gamepad2.y){
            intake.setDirection(Servo.Direction.REVERSE);
            intake.setPosition(-1);
        }

        if(gamepad2.a){
            dump.setPosition(1);
            dump.setDirection(Servo.Direction.FORWARD);
        }

        //Move Linear Slide Up
        if (gamepad2.left_stick_y>0) {
            linearslide.setPosition(1);
        } else {
            linearslide.setPosition(0);
        }

        //Move Linear Slide Down
        if (gamepad2.left_stick_y<0) {
            linearslide.setPosition(0);
        } else {
            linearslide.setPosition(1);
        }

        //Intake Mechanism Pickup
        if (gamepad2.x) {
            intake.setPosition(1);
        }

        //Dumping On Top Of Linear Slide
        if (gamepad2.y) {
            dump.setPosition(-1);
        }

    }}
