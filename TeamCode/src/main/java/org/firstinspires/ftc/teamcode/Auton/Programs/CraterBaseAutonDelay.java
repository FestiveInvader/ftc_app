package org.firstinspires.ftc.teamcode.Auton.Programs;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auton.DeclarationsAutonomous;

@Autonomous(name="CRATER | REGULAR | DELAY", group="Crater Side")
public class CraterBaseAutonDelay extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        //Remember to time auton so that we don't get in the way of the alliance partner until
        //we absolutely have to place the marker and park.
        super.runOpMode();
        unlatch(20);
        telemetry.update();
        craterSideSample();
        gyroTurn(turningSpeed, -80 );//turn to the left, facing the depot-ish
        //This step should be a function driving to the depot to either wait to place the marker
        //Or to place it and sample the other team's sample
        driveFromCraterAfterSampleToNearDepot(32, 18);//latest is 18, normal should be 5 to avoid any delay
        //sleep here for allowing partner to place
        deployTeamMarker();
        craterSidePark();
        endAuto(true);
    }
}