package org.firstinspires.ftc.teamcode.Auton.Programs;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auton.GameSpecificMovement;

@Autonomous(name="Depot|End in far crater ", group="Depot")
public class DepotBaseAuton extends GameSpecificMovement {

    @Override
    public void runOpMode() {
        super.runOpMode();
        hangingSystem.unlatch();
        depotSideSample();
        depotSideDeployMarker();
        deployTeamMarker();//At this point we'll be on the edge of the depot and about to place the marker
        sleep(250);
        genMovement.encoderDrive(.75, 64, reverse, stayOnHeading, 5);
        endAuto(true);//Housekeeping, make sure slide is down, etc.
        //Potentially make a depot option that will go park in our crater side by going under the lander?
        //It'd be sweet
    }
}