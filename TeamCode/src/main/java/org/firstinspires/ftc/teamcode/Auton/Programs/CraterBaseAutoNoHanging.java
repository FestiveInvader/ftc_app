package org.firstinspires.ftc.teamcode.Auton.Programs;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auton.GameSpecificMovement;

@Autonomous(name="Crater side | no hang ", group="Crater Side")
public class CraterBaseAutoNoHanging extends GameSpecificMovement {
    @Override
    public void runOpMode() {
        super.runOpMode();
        //unlatch();
        craterSideSample();
        endAuto();
    }
}