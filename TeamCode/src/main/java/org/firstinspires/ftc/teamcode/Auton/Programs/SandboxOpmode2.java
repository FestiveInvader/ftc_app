package org.firstinspires.ftc.teamcode.Auton.Programs;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auton.DeclarationsAutonomous;

@Autonomous(name="Sandbox 2", group="Test")
public class SandboxOpmode2 extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        gyroTurn(turningSpeed, 90);
        telemetry.addData("rot", getHeading());
        telemetry.update();
        sleep(5000);

    }
}