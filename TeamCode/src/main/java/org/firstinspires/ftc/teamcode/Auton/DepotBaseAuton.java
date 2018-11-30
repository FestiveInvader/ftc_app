package org.firstinspires.ftc.teamcode.Auton;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Depot side | Basic ", group="Depot")
public class DepotBaseAuton extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        //unlatch();
        depotSideSample();
        depotSideDeployMarker();
        depotTurnToFarCrater();
        encoderDrive(.75, 64, reverse, stayOnHeading, 5);
        endAuto();
    }
}