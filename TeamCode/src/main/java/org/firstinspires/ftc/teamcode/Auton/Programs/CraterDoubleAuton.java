package org.firstinspires.ftc.teamcode.Auton.Programs;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auton.DeclarationsAutonomous;

@Autonomous(name="Crater|End in near crater ", group="Crater Side")
public class CraterDoubleAuton extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        unlatch(18);
        craterSideSample();
        //It would be best to figure out the crater base auton first since they'll be based off the
        //same paths, so it'd be safer to figure out the base 85 first.  Plus it's almost exactly the same movement
        endAuto(true);
    }
}