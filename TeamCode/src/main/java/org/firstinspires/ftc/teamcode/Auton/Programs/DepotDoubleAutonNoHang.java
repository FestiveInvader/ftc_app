package org.firstinspires.ftc.teamcode.Auton.Programs;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auton.DeclarationsAutonomous;

@Autonomous(name="Depot side | Double sample no hang ", group="Depot")
public class DepotDoubleAutonNoHang extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        //unlatch();
        depotSideSample();
        depotSideDeployMarker();
        depotTurnToCloseCrater();
        deployTeamMarker();
        depotSideDoubleSample();
        endAuto();
    }
}