package org.firstinspires.ftc.teamcode.Auton.Programs;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auton.DeclarationsAutonomous;

@Autonomous(name="Auton Prep", group="Test")
public class SandboxOpmode2 extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
       unlatch(4);
       TeamMarker.setPosition(teamMarkerResting);
        telemetry.addData("Stopped", 1);
        telemetry.update();
        setIntakePower(.7);
        telemetry.addData("First try", 1);
        telemetry.update();
        sleep(2000);
        telemetry.addData("second try", 1);
        telemetry.update();
        while (opModeIsActive()){
            setIntakePower(.7);
        }



    }
}