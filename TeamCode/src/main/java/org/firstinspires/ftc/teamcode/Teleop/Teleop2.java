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

package org.firstinspires.ftc.teamcode.Teleop;

//import com.qualcomm.hardware.motors.TetrixMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo.Direction;
//import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and P+ste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Default: Iterative OpMode", group="Iterative OpMode")
//@Disabled
public class Teleop2 extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime _runtime = new ElapsedTime();
    /*private DcMotor _leftFrontMotor;
    private DcMotor _rightFrontMotor;
    private DcMotor _rightBackMotor;
    private DcMotor _leftBackMotor;
    private DcMotor _arm;
    private DcMotor _leftArm;
    private DcMotor _screw;
    *///private Servo _clampServo;
    private CRServo _continuousServo;
    private CRServo _wristServo;
    //private DcMotor _spoolMotor;
    //private static final double continuous_servo_stop = 0.5;
    //private static final double continuous_servo_forward = 1.0;
    //private static final double continuous_servo_reverse = 0.0;

    /*
     * private Servo _rightServo;
     */

    /*private boolean _isLeftBumperReleased = true;
    private boolean _isclampServoOpen = false;

    private boolean _isRightBumperReleased = true;
    private boolean _isRightServoOpen = false;
 */
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
//        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
      /* _leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
       _leftBackMotor = hardwareMap.dcMotor.get("leftBack");
       _rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
       _rightBackMotor = hardwareMap.dcMotor.get("rightBack");
       _arm = hardwareMap.dcMotor.get("arm");
       _leftArm = hardwareMap.dcMotor.get("leftArm");
       _screw = hardwareMap.dcMotor.get("leadScrewDrive");
      */ //_clampServo = hardwareMap.get(Servo.class, "clampServo");
        _continuousServo = hardwareMap.get(CRServo.class, "continuousServo");
        //_wristServo = hardwareMap.servo.get("wristServo");
        _wristServo = hardwareMap.get(CRServo.class, "wristServo");
        //_spoolMotor = hardwareMap.dcMotor.get("spoolMotor");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
       /*_leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
       _leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
       _rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
       _rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
       _arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       _leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


       _leftFrontMotor.setPower(0);
       _leftBackMotor.setPower(0);
       _rightFrontMotor.setPower(0);
       _rightBackMotor.setPower(0);
       _spoolMotor.setPower(0);

       /*
       _leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       _rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


       // Tell the driver that initialization is complete.
//        telemetry.addData("Status", "Initialized");
       _leftServo = hardwareMap.get(Servo.class, "left servo");
       _rightServo = hardwareMap.get(Servo.class, "right servo");
       */

    }


    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        _runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        _runtime.startTime();
        // Setup a variable for each drive wheel to save power level for telemetry
       /*double leftPower = gamepad1.left_stick_y;
       double rightPower = gamepad1.right_stick_y;
       boolean servoPowerLeft = gamepad2.left_bumper;
       boolean servoPowerRight = gamepad2.right_bumper;

       telemetry.addLine("Wrist Servo: " + _wristServo.getPower());
       telemetry.addLine("LeftPower: " + leftPower);
       telemetry.addLine("RightPower: " + rightPower);
       //telemetry.addLine("Servo position " + _clampServo.getPosition());
       telemetry.update();*/


//        if(gamepad2.left_bumper == true){
//           _clampServo.setPosition(.5);
//       }
//       if(gamepad2.right_bumper == true){
//            _clampServo.setPosition(1);
//        }
        //clamp motor = team mark motor

       /*
       if(gamepad2.left_trigger > 0)
       {
           _screw.setPower(gamepad2.left_trigger);
       }
       else if(gamepad2.right_trigger > 0)
       {
           _screw.setPower(-gamepad2.right_trigger);
       }
       else
       {
           _screw.setPower(0.0);
       }

       if(gamepad1.left_trigger > 0)
       {
           _arm.setPower(gamepad1.left_trigger);
           _leftArm.setPower(-gamepad1.left_trigger);
       }
       else if(gamepad1.right_trigger > 0)
       {

           _arm.setPower(-gamepad1.right_trigger);
           _leftArm.setPower(gamepad1.right_trigger);
       }
       else
       {
           _arm.setPower(0.0);//arm = right arm motor
           _leftArm.setPower(0.0);
           _arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
           _leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       }
       /*if(servoPowerLeft==true){
          _clampServo.setPosition(.5);
       }

       if(servoPowerRight==true){
           _clampServo.setPosition(1);
       }
       */
       /*
       _leftFrontMotor.setPower(leftPower);
       _leftBackMotor.setPower(leftPower);
       _rightBackMotor.setPower(rightPower);
       _rightFrontMotor.setPower(rightPower);
       */
        /*//DO NOT CHANGE ANY OF THE FOLLOWING CODE !!!
        OG code
        _continuousServo.setPower(-.946);

        if(gamepad2.right_stick_y < 0){
            _continuousServo.setPower(1);
        }
        else if(gamepad2.right_stick_y > 0){
            _continuousServo.setPower(0);
        }
        else{
            _continuousServo.setPower(-.946);
        }
        //DO NOT CHANGE ANY PART OF THE ABOVE CODE !!!

        //flap for shovel and collector servo
        _wristServo.setPower(_continuousServo.getPower());

        if(gamepad2.left_stick_y < 0){
            _wristServo.setPower(1);
        }
        else if(gamepad2.left_stick_y > 0){
            _wristServo.setPower(0);
        }
        else{
            _wristServo.setPower(-.946);
        }*/
//DO NOT CHANGE ANY OF THE FOLLOWING CODE !!!

        //Instead of using 0 for the comparison, it's a good idea to move slightly away from that, hence the .02 change I've made

        if(gamepad2.right_stick_y < -.02){//if GP2 RJY is moved forward, rotate the CS forward
            _continuousServo.setPower(1);
        }else if(gamepad2.right_stick_y > .02){//if GP2 RJY is moved Backwards, don't move _continuousServo
            _continuousServo.setPower(0);
        }else{//Else if the right joystick is at 0, set CS to reverse
            _continuousServo.setPower(-.946);
        }
        //DO NOT CHANGE ANY PART OF THE ABOVE CODE !!!

        //flap for shovel and collector servo
        if(gamepad2.left_stick_y < -.02){//if GP2 LJY is moved forward, rotate the wrist servo forward
            _wristServo.setPower(1);
        }else if(gamepad2.left_stick_y > .02){//if GP2 LJY is moved backward, keep the wrist servo where it is
            _wristServo.setPower(0);
        }else{//If the Left joystick hasn't been moved, set servo to reverse
            _wristServo.setPower(-.946);
        }

       /*
       if(gamepad1.x){
           _spoolMotor.setPower(-1.0);
       }
       else if (gamepad1.y){
           _spoolMotor.setPower(1.0);
       }
       else{
           _spoolMotor.setPower(0);
           _spoolMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior. BRAKE);
       }
       */
        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        // leftPower    = Range.clip(gamepad1.left_stick_y, -1.0, 1.0) ;
        // rightPower   = Range.clip(gamepad1.right_stick_y, -1.0, 1.0) ;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        //  _leftDrive.setPower(leftPower);
        //  _rightDrive.setPower(rightPower);

//
//        if(gamepad2.right_trigger > 0.0)
//
//        {
//            _arm.setPower(0.3);
//        }
//        else if(gamepad2.left_trigger > 0.0)
//        {
//            _arm.setPower(-0.3);
//        }
//        else
//        {
//            _arm.setPower(0.0);
//        }
//
    /*  if(gamepad2.left_bumper && _isLeftBumperReleased)
      {
           _isclampServoOpen = !_isclampServoOpen;
           _isLeftBumperReleased = false;
          if(_isclampServoOpen)
          {
               _clampServo.setPosition(0.5);
           }
           else{
               _clampServo.setPosition(0.0);
           }
       }
       else if(!_isLeftBumperReleased && !gamepad2.left_bumper)
       {
           _isLeftBumperReleased = true;
       } */
//
//        if(gamepad2.right_bumper && _isRightBumperReleased)
//        {
//            _isRightServoOpen = !_isRightServoOpen;
//            _isRightBumperReleased = false;
//            if(_isRightServoOpen)
//            {
//                _rightServo.setPosition(0.5);
//            }
//            else
//            {
//                _rightServo.setPosition(0.0);
//            }
//        }
//        else if(!_isRightBumperReleased && !gamepad2.right_bumper)
//        {
//            _isRightBumperReleased = true;
//        }

        // Show the elapsed game time and wheel power.
//        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("Left Joystick position", gamepad2.left_stick_y);
        telemetry.addData("Right Joystick position", gamepad2.right_stick_y);
        telemetry.addData("CS power", _continuousServo.getPower());
        telemetry.addData("Wrist Servo Power", _wristServo.getPower());
        telemetry.update();
        //This will just be overridden by the if else loop's setting their power, so it's unneccessary
        _continuousServo.setPower(-.946);
        _wristServo.setPower(_continuousServo.getPower());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */

    @Override
    public void stop() {
     /*  _leftFrontMotor.setPower(0);
       _leftBackMotor.setPower(0);
       _rightFrontMotor.setPower(0);
       _rightBackMotor.setPower(0);
       _arm.setPower(0);
       _leftArm.setPower(0);
       _arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       _leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       //_wristServo.setPower(-.946);*/
    }

}

