package org.firstinspires.ftc.teamcode.Auton.Programs;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auton.GameSpecificMovement;

@Autonomous(name="Sandbox ", group="Test")
public class SandboxOpmode extends GameSpecificMovement {
    @Override
    public void runOpMode() {
        super.runOpMode();
        int loops = 0;
        genMovement.encoderDrive(.3, 10 ,forward, stayOnHeading,10);
        //endAuto();
    }
}