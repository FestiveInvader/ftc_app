package org.firstinspires.ftc.teamcode.Auton.Programs;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auton.GameSpecificMovement;

@Autonomous(name="Crater side | no team marker ", group="Crater Side")
public class CraterBaseAuton extends GameSpecificMovement {
    @Override
    public void runOpMode() {
        super.runOpMode();
        hangingSystem.unlatch();
        craterSideSample();
        endAuto();
    }
}