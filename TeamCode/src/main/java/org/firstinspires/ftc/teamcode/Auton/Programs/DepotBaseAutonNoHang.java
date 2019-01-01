package org.firstinspires.ftc.teamcode.Auton.Programs;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auton.DeclarationsAutonomous;

@Autonomous(name="Depot side | Basic No Hang", group="Depot")
public class DepotBaseAutonNoHang extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        //unlatch();
        depotSideSample();
        depotSideDeployMarker();
        depotTurnToFarCrater();
        deployTeamMarker();
        encoderDrive(.75, 64, reverse, stayOnHeading, 5);
        endAuto();
    }
}