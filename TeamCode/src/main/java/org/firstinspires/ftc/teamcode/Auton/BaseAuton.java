package org.firstinspires.ftc.teamcode.Auton;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name="Auton Test ", group="Test")
public class BaseAuton extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        int loops = 0;
        encoderDrive(.3, 54,forward,stayOnHeading,10);
        //endAuto();
    }
}