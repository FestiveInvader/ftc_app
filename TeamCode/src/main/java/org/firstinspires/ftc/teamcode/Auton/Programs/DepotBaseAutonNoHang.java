package org.firstinspires.ftc.teamcode.Auton.Programs;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auton.GameSpecificMovement;

@Autonomous(name="Depot side | Basic No Hang", group="Depot")
public class DepotBaseAutonNoHang extends GameSpecificMovement {
    @Override
    public void runOpMode() {
        super.runOpMode();
        //unlatch();
        depotSideSample();
        depotSideDeployMarker();
        depotTurnToFarCrater();
        deployTeamMarker();
        genMovement.encoderDrive(.75, 64, reverse, stayOnHeading, 5);
        endAuto();
    }
}