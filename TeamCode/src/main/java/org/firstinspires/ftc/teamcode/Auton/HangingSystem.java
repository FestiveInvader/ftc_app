package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class HangingSystem extends GameSpecificMovement {
    public void unlatch(){

        HangingSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int inchesToUnlatch = 26;
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
    public void keepMineralArmUp(){
        double potRotation = ArmPot.getVoltage()/potMagicNumber;
        double armRotError = (Math.abs(potRotation)-Math.abs(armScoringRotation));

        armPower = Range.clip(armRotError*armPVal, -1, 1);
        ArmTop.setPower(armPower);
        ArmBottom.setPower(armPower);
    }
    public void unextendHangSlide(){
        //this is made so it can be in a loop by itself, or in another loop.
        HangingSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(hangSlideIsExtended()){
            HangingSlide.setPower(.5);
            keepMineralArmUp();
        }else{
            HangingSlide.setPower(0);
            ArmTop.setPower(0);
            ArmBottom.setPower(0);
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
