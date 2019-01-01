package org.firstinspires.ftc.teamcode.Auton;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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