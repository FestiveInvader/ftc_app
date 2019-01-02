package org.firstinspires.ftc.teamcode.Auton.Programs;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auton.DeclarationsAutonomous;

@Autonomous(name="Sandbox 2 ", group="Test")
public class SandboxOpmode2 extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        while (opModeIsActive() && hangSlideIsExtended()) {
            unextendHangSlide(true);
            telemetry.addData("Going down", 1);
            telemetry.update();
        }
        telemetry.addData("Stopped", 1);
        telemetry.update();
        sleep(30000);
    }
}