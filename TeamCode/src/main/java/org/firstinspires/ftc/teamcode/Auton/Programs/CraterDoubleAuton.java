package org.firstinspires.ftc.teamcode.Auton.Programs;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auton.DeclarationsAutonomous;

@Autonomous(name="Crater|Double sample", group="Crater Side")
public class CraterDoubleAuton extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        //Remember to time auton so that we don't get in the way of the alliance partner until
        //we absolutely have to place the marker and park.
        super.runOpMode();
        unlatch(18);
        craterSideSample();
        gyroTurn(turningSpeed, -80 );
        driveFromCraterAfterSampleToNearDepot(18, 22);
        craterDoubleSample();
        endAuto(true);
    }
}