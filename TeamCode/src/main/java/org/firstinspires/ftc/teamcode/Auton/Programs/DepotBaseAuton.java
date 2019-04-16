package org.firstinspires.ftc.teamcode.Auton.Programs;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auton.DeclarationsAutonomous;

@Autonomous(name="DEPOT | ENDS IN FAR CRATER ", group="Depot")
public class DepotBaseAuton extends DeclarationsAutonomous {

    @Override
    public void runOpMode() {
        super.runOpMode();
        unlatch(20);
        intakeSample();
        gyroTurn(turningSpeed, -80 );//turn to the left, facing the crater-ish
        depotSideDeployAndPark();
        endAuto(false);//Housekeeping, make sure slide is down, etc.
        //Potentially make a depot option that will go park in our crater side by going under the lander?
        //It'd be sweet}
        encoderDrive(.25, 2, forward,stayOnHeading,3,false);
    }
}