package org.firstinspires.ftc.teamcode.Auton;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Depot side | Basic ", group="Depot Side")
public class DepotBaseAuton extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        //unlatch();
        depotSideSample();
        depotSideDeployMarker();
        depotDriveToFarCrater();
        endAuto();
    }
}