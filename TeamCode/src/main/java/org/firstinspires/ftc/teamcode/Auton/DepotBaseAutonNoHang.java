package org.firstinspires.ftc.teamcode.Auton;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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