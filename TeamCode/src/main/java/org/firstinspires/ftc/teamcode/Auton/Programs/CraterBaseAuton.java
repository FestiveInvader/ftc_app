package org.firstinspires.ftc.teamcode.Auton.Programs;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auton.GameSpecificMovement;

@Autonomous(name="Crater side", group="Crater Side")
public class CraterBaseAuton extends GameSpecificMovement {
    @Override
    public void runOpMode() {
        //Remember to time auton so that we don't get in the way of the alliance partner until
        //we absolutely have to place the marker and park.
        super.runOpMode();
        hangingSystem.unlatch();
        craterSideSample();
        genMovement.gyroTurn(turningSpeed, -90);//turn to the left, facing the depot-ish
        //This step should be a function driving to the depot to either wait to place the marker
        //Or to place it and sample the other team's sample
        driveFromCraterAfterSampleToNearDepot();
        //sleep here for allowing partner to place
        genMovement.encoderDrive(.35, 24, forward, stayOnHeading, 1);
        deployTeamMarker();
        craterSideParkArmInCrater();
        endAuto(false);
    }
}