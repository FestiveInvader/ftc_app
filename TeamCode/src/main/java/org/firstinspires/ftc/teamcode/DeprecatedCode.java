/*
package org.firstinspires.ftc.teamcode;

public class DeprecatedCode {
     */
/* public void depotSideDoubleSample(){
         goToDistance(.35, 100, FrontDistance,5,3);
         gyroTurn(turningSpeed, -30);
         encoderDrive(.75, 18, reverse, stayOnHeading, 3);
         gyroTurn(turningSpeed, 18);
         encoderDrive(.75, 20, reverse, stayOnHeading, 3);
         gyroTurn(turningSpeed, 90);
         encoderDrive(.2, 4, reverse, stayOnHeading, 1.25);
         gyroTurn(turningSpeed, decideSecondSampleheading());
         if(goldPosition == 2){
             encoderDrive(.5, 16, forward, stayOnHeading, 2.5);
         }else{
             encoderDrive(.5, 24, forward, stayOnHeading, 2.5);
         }
     }*//*

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
                unextendHangSlide(true);
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
    public void goToDistance(double targetSpeed, double distance,  I2CXLv2 distanceSensor, double timeout, int tolerance){
        double startTime = runtime.seconds();
        double maxTime = startTime + timeout;
        double startHeading = getHeading();
        boolean foundTarget = false;
        int ThisLoopDistance;
        while (opModeIsActive() && !foundTarget && maxTime - runtime.seconds() > .1) {
            ThisLoopDistance = FrontDistance.getDistance();
            double error = distance - ThisLoopDistance;
            int Direction = (int) -Range.clip(error, -1, 1);
            if(ThisLoopDistance > 500 || ThisLoopDistance < 21){
                //gyroDrive(startHeading, Range.clip(Math.abs(error/175), .2, targetSpeed), Direction);
                //sensor val is bad, stop bot so it doesn't go too far
            }else if(ThisLoopDistance > distance + tolerance || ThisLoopDistance < distance - tolerance){
                gyroDrive(startHeading, Range.clip(Math.abs(error/175), .2, targetSpeed), Direction);
            }else{
                stopDriveMotors();
                foundTarget = true;
            }
            telemetry.addData("Distance", ThisLoopDistance);
            telemetry.addData("error", error);
            telemetry.addData("speed", LeftTop.getPower());
            telemetry.update();
        }
        stopDriveMotors();
    }
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
boolean onHeading(double speed, double angle, double PCoeff) {
        //This function is a boolean, meaning it can be used in an if/while statement. For instance:
        //while(!onHeading){} would run all this code, until onHeading returns true
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;
        double PosOrNeg;
        double error = getError(angle);
        double minTurnSpeed = .275;//definitely play with this val
        double maxTurnSpeed = .75;
        // determine turn power based on +/- error
        //unextendHangSlide(true);
        keepMineralArmUp();
        if (Math.abs(error) <= .25) {
            steer = 0.0;
            leftSpeed  = 0;
            rightSpeed = 0;
            onTarget = true;
        }
        else {
            if(IMU.getAngularVelocity().zRotationRate < .4){
                //we're spinning too slow to turn now
                minTurnSpeed += .02;
            }
            // This is the main part of the Proportional GyroTurn.  This takes a set power variable,
            // and adds the absolute value of error/150.  This allows for the robot to turn faster when farther
            // away from the target, and turn slower when closer to the target.  This allows for quicker, as well
            // as more accurate turning when using the GyroSensor
            PosOrNeg = Range.clip((int)error, -1, 1);
            //steer = getSteer(error, PCoeff);

            leftSpeed  = Range.clip(minTurnSpeed + Math.abs(error)/250 , minTurnSpeed, maxTurnSpeed)* PosOrNeg;
            rightSpeed = -leftSpeed;

        }

        // Set motor speeds.
        LeftTop.setPower(leftSpeed);
        LeftBottom.setPower(leftSpeed);
        RightTop.setPower(rightSpeed);
        RightBottom.setPower(rightSpeed);
        // Display debug info in telemetry.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Speeds, Left,Right.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
        telemetry.addData("turning speed.",  IMU.getAngularVelocity().zRotationRate);
        return onTarget;
    }public void neutralSideSample(){
        double heading = getHeading();
        ElapsedTime elapsedTime = new ElapsedTime();
//should come immediately after unlatching
        //for the most part this should be able to be copy/pasted to the depotSideSample, though a few changes
        //for the team marker may have to be made.

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
        gyroTurn(turningSpeed, decideSecondSampleheading());
        if(goldPosition == 2){
            encoderDrive(.5, 12, forward, stayOnHeading, 5, true);
        }else{
            encoderDrive(.5, 18, forward, stayOnHeading, 5, true);
        }
    }
public void depotTurnToFarCrater(){
        //This will drive to the other alliance's side's crater, to be out of the way of our partner's team marker.
        //We should also have a version that goes to our side, if our alliance partner also scores in the lander (so that
        // we get a little bit of extra time for cycles.
        gyroTurn(turningSpeed, -45);
        encoderDrive(.425, 18, forward, stayOnHeading, 2, true);
        double thisHeading = getHeading();
        encoderDrive(.35, 2.5, reverse, stayOnHeading, 1.5, true);
        TeamMarker.setPosition(teamMarkerResting);
        double turningHeading = -thisHeading + 91;
        gyroTurn(turningSpeed, turningHeading);
    }
    public void depotTurnToCloseCrater(){
        //This will drive to the other alliance's side's crater, to be out of the way of our partner's team marker.
        //We should also have a version that goes to our side, if our alliance partner also scores in the lander (so that
        // we get a little bit of extra time for cycles.
        gyroTurn(turningSpeed, 45);
        encoderDrive(.425, 18, forward, stayOnHeading, 2, true);
        double thisHeading = getHeading();
        encoderDrive(.35, 2, reverse, stayOnHeading, 1.5, true);
        TeamMarker.setPosition(teamMarkerResting);
        double turningHeading = -thisHeading - 89;
        gyroTurn(turningSpeed, turningHeading);
    }
    public void driveFromDepot(){}

    public void getGoldPositionTwoMineral(){
        if (goldPosition == 0 && opModeIsActive()) {
            if (tfod != null) {
                tfod.activate();

                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 2) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }
                        if(goldMineralX == -1){
                            telemetry.addData("Gold Mineral Position", "Left");
                            goldPosition = 1;
                        }else if(goldMineralX < silverMineral1X){
                            telemetry.addData("Gold Mineral Position", "Center");
                            goldPosition = 2;
                        }else{
                            telemetry.addData("Gold Mineral Position", "Right");
                            goldPosition = 3;
                        }
                        telemetry.addData("Gold pos", goldPosition);
                        telemetry.addData("Gold X pos", goldMineralX);
                        telemetry.addData("Silver1 X pos", silverMineral1X);
                        telemetry.addData("Silver2 X pos", silverMineral2X);
                        telemetry.update();
                    }
                }
            }
        }
    }
        /*double timeout = 1.5;
        double time = runtime.seconds();
        // keep looping while we are still active, and not on heading.
        //uses onHeading to actually turn the robot/figure out error
            while (opModeIsActive() && !onHeading(speed, -angle, P_TURN_COEFF)) {
                // Update telemetry & Allow time for other processes to run.
                telemetry.addData("Target Rot", angle);
                telemetry.addData("Current Rot", getHeading());
                telemetry.update();
            }
            ArmTop.setPower(0);
            ArmBottom.setPower(0);


                public void pivot(double rotationSpeed, int direction){
        LeftTop.setPower(rotationSpeed*-direction);
        LeftBottom.setPower(rotationSpeed*-direction);
        RightTop.setPower(rotationSpeed*direction);
        RightBottom.setPower(rotationSpeed*direction);
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * (PCoeff), -1, 1);
    }


*/
