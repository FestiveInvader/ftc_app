package org.firstinspires.ftc.teamcode.Auton;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Crater side | no team marker ", group="Crater Side")
public class CraterBaseAuton extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        unlatch();
        craterSideSample();
        endAuto();
    }
}