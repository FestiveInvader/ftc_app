package org.firstinspires.ftc.teamcode.Auton.Programs;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auton.GameSpecificMo;

@Autonomous(name="Depot side | Double sample ", group="Depot")
public class DepotDoubleAuton extends GameSpecificMo {
    @Override
    public void runOpMode() {
        super.runOpMode();
        unlatch();
        depotSideSample();
        depotSideDeployMarker();
        depotTurnToCloseCrater();
        deployTeamMarker();
        depotSideDoubleSample();
        endAuto();
    }
}