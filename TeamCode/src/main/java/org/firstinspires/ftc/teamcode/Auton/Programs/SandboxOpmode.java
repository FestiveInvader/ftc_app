package org.firstinspires.ftc.teamcode.Auton.Programs;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auton.DeclarationsAutonomous;

@Autonomous(name="Test intake servos ", group="Test")
public class SandboxOpmode extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        IntakeLeft.setPower(.7);
        sleep(2000);
        IntakeLeft.setPower(0);
        sleep(1000);
        IntakeRight.setPower(.7);
        sleep(2000);
        IntakeFlapLeft.setPosition(1);
    }
}