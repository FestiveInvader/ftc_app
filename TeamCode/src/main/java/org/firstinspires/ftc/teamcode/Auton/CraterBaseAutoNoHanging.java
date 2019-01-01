package org.firstinspires.ftc.teamcode.Auton;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Crater side | no hang ", group="Crater Side")
public class CraterBaseAutoNoHanging extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        //unlatch();
        craterSideSample();
        endAuto();
    }
}