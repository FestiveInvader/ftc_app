package org.firstinspires.ftc.teamcode.Auton.Programs;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auton.GameSpecificMovement;

@Autonomous(name="Depot side | Basic ", group="Depot")
public class DepotBaseAuton extends GameSpecificMovement {

    @Override
    public void runOpMode() {
        super.runOpMode();
        hangingSystem.unlatch();
        depotSideSample();
        depotSideDeployMarker();
        deployTeamMarker();
        genMovement.encoderDrive(.75, 64, reverse, stayOnHeading, 5);
        endAuto();
    }
}