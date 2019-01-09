package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class ScoringSystems extends DeclarationsAutonomous{

    public void putArmDown() {
        while(opModeIsActive() && !armIsDown()) {
            putMineralArmDown();
        }
        ArmTop.setPower(0);
        ArmBottom.setPower(0);
    }
    public boolean armIsDown() {
        if (potRotation() < 130) {
            return false;
        } else {
            return true;
        }
    }
    public void keepMineralArmUp(){
        double armRotError = (Math.abs(potRotation())-Math.abs(armScoringRotation));

        armPower = Range.clip(armRotError*armPVal, -1, 1);
        ArmTop.setPower(armPower);
        ArmBottom.setPower(armPower);
    }
    public void putMineralArmDown(){
        double armRotError = (Math.abs(potRotation())-Math.abs(armDownRotation));

        armPower = Range.clip(armRotError*armPVal, -.5, .5);
        ArmTop.setPower(armPower);
        ArmBottom.setPower(armPower);
    }

    public void setIntakePower(double power){
        Range.clip(power, -.7, .7);//393s have a power limit of .7.  Higher and they won't spin
        IntakeLeft.setPower(power);
        IntakeRight.setPower(power);
    }
    public double potRotation(){
        double potRotation = ArmPot.getVoltage()/potMagicNumber;
        return potRotation;
    }
    public void extendMineralArm(int inchesToExtend){
        ArmSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmSlide.setTargetPosition((int) ticksToExtendMineralArmInch * inchesToExtend);
        double timer = runtime.seconds() + 1;
        while(timer > runtime.seconds() && opModeIsActive() && runtime.seconds() < 27){
            keepMineralArmUp();
        }
        while(ArmSlide.isBusy() && opModeIsActive() &&  runtime.seconds() < 28){
            keepMineralArmUp();
            ArmSlide.setPower(1);
        }
        ArmSlide.setPower(0);
    }
    public void unlatch(int inchesToUnlatch){

        HangingSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        HangingSlide.setTargetPosition((int) ticksPerHangingInch * -inchesToUnlatch);
        HangCamLeft.setPosition(hangCamLeftUnengagedPos);
        HangCamRight.setPosition(hangCamRightUnengagedPos);
        double timer = runtime.seconds() + 1;
        while(timer > runtime.seconds() && opModeIsActive()){
            keepMineralArmUp();
        }
        while(HangingSlide.isBusy()){
            HangingSlide.setPower(1);
            keepMineralArmUp();
        }
        HangingSlide.setPower(0);
        ArmTop.setPower(0);
        ArmBottom.setPower(0);
    }

    public void unextendHangSlide(boolean keepArmUp){
        //this is made so it can be in a loop by itself, or in another loop.
        HangingSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        HangingSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if(!hangSlidesDown) {
            if (hangSlideIsExtended()) {
                HangingSlide.setPower(.5);
            } else {
                HangingSlide.setPower(0);
                hangSlidesDown = true;
            }
        }
        if (keepArmUp) {
            keepMineralArmUp();
        }
    }
    public boolean hangSlideIsExtended(){
        if(HangSlideLimit.getState() == false){
            return false;
        }else {
            return true;
        }
    }


}
