package org.firstinspires.ftc.teamcode.Auton;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Auton/Programs/DepotBaseAuton.java
import org.firstinspires.ftc.teamcode.Auton.GameSpecificMovement;

@Autonomous(name="Depot|End in far crater ", group="Depot")
public class DepotBaseAuton extends GameSpecificMovement {

=======
@Autonomous(name="Depot side | Basic ", group="Depot")
public class DepotBaseAuton extends DeclarationsAutonomous {
>>>>>>> parent of 2fc527b... Organization of auton:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Auton/DepotBaseAuton.java
    @Override
    public void runOpMode() {
        super.runOpMode();
        unlatch();
        depotSideSample();
        depotSideDeployMarker();
<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Auton/Programs/DepotBaseAuton.java
        deployTeamMarker();//At this point we'll be on the edge of the depot and about to place the marker
        sleep(250);
        genMovement.encoderDrive(.75, 64, reverse, stayOnHeading, 5);
        endAuto(true);//Housekeeping, make sure slide is down, etc.
        //Potentially make a depot option that will go park in our crater side by going under the lander?
        //It'd be sweet
=======
        depotTurnToFarCrater();
        deployTeamMarker();
        encoderDrive(.75, 64, reverse, stayOnHeading, 5);
        endAuto();
>>>>>>> parent of 2fc527b... Organization of auton:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Auton/DepotBaseAuton.java
    }
}