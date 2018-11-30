package org.firstinspires.ftc.teamcode.Auton;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name="Auton Test ", group="Test")
public class BaseAuton extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        int loops = 0;
        goToDistance(.5, 190, FrontDistance, 5, 2);

        //endAuto();
    }
}