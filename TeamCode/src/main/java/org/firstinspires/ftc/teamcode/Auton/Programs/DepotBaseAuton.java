package org.firstinspires.ftc.teamcode.Auton.Programs;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auton.DeclarationsAutonomous;

@Autonomous(name="DEPOT | ENDS IN FAR CRATER ", group="Depot")
public class DepotBaseAuton extends DeclarationsAutonomous {

    @Override
    public void runOpMode() {
        super.runOpMode();
        unlatch(20);
        depotSideSample();
        depotSideDeployAndPark();
        endAuto(true);//Housekeeping, make sure slide is down, etc.
        //Potentially make a depot option that will go park in our crater side by going under the lander?
        //It'd be sweet}
    }
}