package org.firstinspires.ftc.teamcode.Auton.Programs;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Auton.DeclarationsAutonomous;

@Autonomous(name="tEsT", group="Test")
public class SandboxOpmode2 extends DeclarationsAutonomous {
    @Override
    public void runOpMode() {
        super.runOpMode();
        gyroTurn(turningSpeed, 178);
        while(opModeIsActive()){
            telemetry.addData("IMU heading0", getHeading());
            telemetry.update();
        }
    }
}