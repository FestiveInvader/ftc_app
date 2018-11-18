package org.firstinspires.ftc.teamcode.Auton;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name="Unlatch, probably", group="Auton")
public class BaseAuton extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        unlatch();
        while(hangSlideIsExtended()&& opModeIsActive()){
            unextendHangSlide();
        }
        endAuto();
    }
}