package org.firstinspires.ftc.teamcode.Auton;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Depot side | Double sample ", group="Depot")
public class DepotDoubleAuton extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        //unlatch();
        depotSideSample();
        depotSideDeployMarker();
        depotTurnToCloseCrater();
        deployTeamMarker();
        depotSideDoubleSample();
        TeamMarker.setPosition(teamMarkerResting);
        //endAuto();
    }
}