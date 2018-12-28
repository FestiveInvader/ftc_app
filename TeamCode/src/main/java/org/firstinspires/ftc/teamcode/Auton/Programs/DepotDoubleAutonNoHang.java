package org.firstinspires.ftc.teamcode.Auton.Programs;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auton.GameSpecificMo;

@Autonomous(name="Depot side | Double sample no hang ", group="Depot")
public class DepotDoubleAutonNoHang extends GameSpecificMo {
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