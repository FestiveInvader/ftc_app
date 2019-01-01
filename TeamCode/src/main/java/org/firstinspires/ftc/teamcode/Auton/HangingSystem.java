package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class HangingSystem extends GameSpecificMovement {

    boolean hangRatchetEngaged = true;
    double hangCamLeftEngagedPos = 1;
    double hangCamLeftUnengagedPos = 0;
    double hangCamRightEngagedPos = 0;
    double hangCamRightUnengagedPos = 1;

    double armScoringRotation = 65;
    double armDownRotation = 160;
    double armPVal = .025;
    double armPower;
    public boolean slideExtended;
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
        slideExtended = true;
    }
    public void keepMineralArmUp(){
        double potRotation = ArmPot.getVoltage()/potMagicNumber;
        double armRotError = (Math.abs(potRotation)-Math.abs(armScoringRotation));

        armPower = Range.clip(armRotError*armPVal, -1, 1);
        ArmTop.setPower(armPower);
        ArmBottom.setPower(armPower);
    }
    public void putMineralArmDown(){
        double potRotation = ArmPot.getVoltage()/potMagicNumber;
        double armRotError = (Math.abs(potRotation)-Math.abs(armDownRotation));

        armPower = Range.clip(armRotError*armPVal, -1, 1);
        ArmTop.setPower(armPower);
        ArmBottom.setPower(armPower);
    }
    public  void unextendHangSlide(){
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
    public  boolean hangSlideIsExtended(){
        if(HangSlideLimit.getState() == false){
            slideExtended = false;
        }
        return slideExtended;
    }

}
