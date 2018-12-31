package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.teamcode.I2CXLv2;

public class GeneralMovement extends GameSpecificMovement {
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

    public void encoderDrive(double speed, double Inches, int direction, double heading, double timeout) {
        double startTime = runtime.seconds();
        double Heading = 0;
        boolean notAtTarget = true;
        hangingSystem.unextendHangSlide();
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
    public void EncoderDriveAccelDecel(double speed, double inches, double accelInches, double decelInches, int direction, double heading, double timeout){
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
            double power = 0;
            double error;

            target = LeftTop.getCurrentPosition() + (int) (inches * CountsPerInch * direction);
            double accelTicks = (int) (accelInches * CountsPerInch);
            double decelTicks = (int) (decelInches * CountsPerInch);
            //while the absolute value of the target minus the AV of the FL encoder value > 25 {
            //if(AV of TL ecnoders < accelTicks position) {Accel}
            //else if{ AV of TL ecnoders > decel position
            //else {drive normal}
            while(Math.abs(target) - Math.abs(LeftTop.getCurrentPosition()) > 25 && runtime.seconds() < 28.5 && (startTime + timeout > runtime.seconds())) {
                double motorPos = Math.abs(LeftTop.getCurrentPosition());
                hangingSystem.unextendHangSlide();
                error = Math.abs(target) - Math.abs(motorPos);
                if(Math.abs(motorPos) < Math.abs(accelTicks)){
                    //Accel
                    power = Math.abs(LeftTop.getCurrentPosition())/Math.abs(accelTicks);
                    //Should get closer and closer to the max power the closer it gets to total accel ticks
                    gyroDrive(Heading, Range.clip(power, .15, speed), direction);
                    telemetry.addData("TopLeftPwr", LeftTop.getPower());
                    telemetry.addData("In accel", LeftTop.getPower());
                    telemetry.update();
                }else if(Math.abs(motorPos) > Math.abs(decelTicks)){
                    //Decel
                    power = (Math.abs(target) - Math.abs(decelTicks)) / (Math.abs(target) - Math.abs(LeftTop.getCurrentPosition()));
                    //Should get closer and closer to the max power the closer it gets to total accel ticks
                    gyroDrive(Heading, Range.clip(power, .15, speed), direction);
                    telemetry.addData("TopLeftPwr", LeftTop.getPower());
                    telemetry.addData("In Decel", LeftTop.getPower());
                    telemetry.update();
                }else{
                    //drive normal speed
                    gyroDrive(Heading, speed, direction);
                    telemetry.addData("TopLeftPwr", LeftTop.getPower());
                    telemetry.addData("In normal speeeeeeeed", LeftTop.getPower());
                    telemetry.update();
                }

            }
            stopDriveMotors();
        }
    }

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
        double minTurnSpeed = .35;//definetly play with this val
        double maxTurnSpeed = 1;
        // determine turn power based on +/- error
        hangingSystem.unextendHangSlide();
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
            //the error/thispower was 175, changed to 100 for more responsiveness
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
    public void pivot(double rotationSpeed, int direction){
        LeftTop.setPower(rotationSpeed*-direction);
        LeftBottom.setPower(rotationSpeed*-direction);
        RightTop.setPower(rotationSpeed*direction);
        RightBottom.setPower(rotationSpeed*direction);
    }


}
