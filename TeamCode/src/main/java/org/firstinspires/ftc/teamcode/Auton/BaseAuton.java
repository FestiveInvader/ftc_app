package org.firstinspires.ftc.teamcode.Auton;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name="Crater side | Basic ", group="Crater Side")
public class BaseAuton extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        unlatch();
        craterSideSample();
        endAuto();
    }
}