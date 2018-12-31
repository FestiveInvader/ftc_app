package org.firstinspires.ftc.teamcode.Auton.Programs;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auton.GameSpecificMovement;

@Autonomous(name="Crater|End in near crater ", group="Crater Side")
public class CraterDoubleAuton extends GameSpecificMovement {
    @Override
    public void runOpMode() {
        super.runOpMode();
        hangingSystem.unlatch();
        craterSideSample();
        //It would be best to figure out the crater base auton first since they'll be based off the
        //same paths, so it'd be safer to figure out the base 85 first.  Plus it's almost exactly the same movement
        endAuto(true);
    }
}