package org.firstinspires.ftc.teamcode.Auton.Programs;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auton.DeclarationsAutonomous;

@Autonomous(name="Crater side", group="Crater Side")
public class CraterBaseAuton extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        //Remember to time auton so that we don't get in the way of the alliance partner until
        //we absolutely have to place the marker and park.
        super.runOpMode();
        unlatch(26);
        craterSideSample();
        gyroTurn(turningSpeed, -75 );//turn to the left, facing the depot-ish
        //This step should be a function driving to the depot to either wait to place the marker
        //Or to place it and sample the other team's sample
        driveFromCraterAfterSampleToNearDepot();
        //sleep here for allowing partner to place
        deployTeamMarker();
        craterSideParkArmInCrater();
        endAuto(false);
    }
}