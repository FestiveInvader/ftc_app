/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class Hardware
{
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
    double hangingGearRatio = 60/40;
    double ticksPerHangingRev = rev40TicksPerRev*hangingGearRatio;

    double mineralArmSpoolDiameter = 1.78;
    double ticksToExtendMineralArmInch = (rev40TicksPerRev/(mineralArmSpoolDiameter * 3.1415));

    double ticksPerHangingInch =  (ticksPerHangingRev/(hangingPulleyDiameter * 3.1415));

    double HEADING_THRESHOLD = 2;      // As tight as we can make it with an integer gyro
    double P_TURN_COEFF = .2;     // Larger is more responsive, but also less stable
    double P_DRIVE_COEFF = .15;     // Larger is more responsive, but also less stable
    public double turningSpeed = .325;

    double potMagicNumber = .01222;

    boolean hangRatchetEngaged = true;
    double hangCamLeftEngagedPos = 1;
    double hangCamLeftUnengagedPos = 0;
    double hangCamRightEngagedPos = 0;
    double hangCamRightUnengagedPos = 1;

    double armScoringRotation = 55;
    double armDownRotation = 150;
    double armPVal = .015;
    double armPower;

    public double teamMarkerDeploy = -.1;
    public double teamMarkerResting = .3;

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

    HardwareMap hwMap           =  null;

    /* Constructor */
    public Hardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
// This section gets the hardware maps
        // This section gets the hardware maps
        LeftTop = hwMap.dcMotor.get("LeftTop");
        LeftBottom = hwMap.dcMotor.get("LeftBottom");
        RightTop = hwMap.dcMotor.get("RightTop");
        RightBottom = hwMap.dcMotor.get("RightBottom");
        HangingSlide = hwMap.dcMotor.get("HangingSlide");
        ArmTop = hwMap.dcMotor.get("ArmTop");
        ArmBottom = hwMap.dcMotor.get("ArmBottom");
        ArmSlide = hwMap.dcMotor.get("ArmSlide");

        IntakeLeft = hwMap.crservo.get("IntakeLeft");
        IntakeRight = hwMap.crservo.get("IntakeRight");
        HangCamLeft = hwMap.servo.get("HangCamLeft");
        HangCamRight = hwMap.servo.get("HangCamRight");
        TeamMarker = hwMap.servo.get("TeamMarker");

        HangSlideLimit = hwMap.get(DigitalChannel.class, "HangSlideLimit");
        HangSlideLimit.setMode(DigitalChannel.Mode.INPUT);

        ArmPot = hwMap.analogInput.get("ArmPot");
        FrontDistance = hwMap.get(I2CXLv2.class, "FrontDistance");

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
        IMU = hwMap.get(BNO055IMU.class, "IMU");
        IMU.initialize(Bparameters);
        // End Init IMU
        /*telemetry.addData("IMU Init'd", true);
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
            telemetry.addData("IMU", getHeading());
            telemetry.addData("FrontLeft Encoder", LeftTop.getCurrentPosition());
            telemetry.addData("If values are good, then run", 1);
            telemetry.addData("If I don't put the not !waitforstart or whatever here then it'll probably crash", 1);
            telemetry.update();
        }*/
    }
 }

