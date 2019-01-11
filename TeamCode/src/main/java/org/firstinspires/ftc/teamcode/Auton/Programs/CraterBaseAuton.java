package org.firstinspires.ftc.teamcode.Auton.Programs;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auton.DeclarationsAutonomous;

@Autonomous(name="Crater|Regular", group="Crater Side")
public class CraterBaseAuton extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        //Remember to time auton so that we don't get in the way of the alliance partner until
        //we absolutely have to place the marker and park.
        super.runOpMode();
        unlatch(26);
        telemetry.addData("Here", 1);
        telemetry.update();
        craterSideSample();

        telemetry.addData("Here", 2);
        telemetry.update();
        gyroTurn(turningSpeed, -80 );//turn to the left, facing the depot-ish
        //This step should be a function driving to the depot to either wait to place the marker
        //Or to place it and sample the other team's sample

        telemetry.addData("Here", 3);
        telemetry.update();
        driveFromCraterAfterSampleToNearDepot(36, 5);

        telemetry.addData("Here", 4);
        telemetry.update();
        //sleep here for allowing partner to place
        deployTeamMarker();
        craterSidePark();
        endAuto(true);
    }
}