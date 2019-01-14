package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class IMUTurning extends Thread {
    private boolean run = true;
    private double degSec;
    private double prevHeading;
    private BNO055IMU imu;
    private double degsPerSec;
    private double loopTime;
    private OpMode opmode;
    private double loops = 0;
    Orientation angles;



    public IMUTurning(double loopTime, OpMode opmode, BNO055IMU IMU) {
        this.imu = IMU;
        this.opmode = opmode;
        this.loopTime = loopTime;
        BNO055IMU.Parameters Bparameters = new BNO055IMU.Parameters();
        Bparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        Bparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        Bparameters.calibrationDataFile = "BNO055IMUCalibration.json";
        Bparameters.loggingEnabled = true;
        Bparameters.loggingTag = "IMU";
        Bparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

    }


    public void run() {
        BNO055IMU.Parameters Bparameters = new BNO055IMU.Parameters();
        Bparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        Bparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        Bparameters.calibrationDataFile = "BNO055IMUCalibration.json";
        Bparameters.loggingEnabled = true;
        Bparameters.loggingTag = "IMU";
        Bparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = opmode.hardwareMap.get(BNO055IMU.class, "IMU");
        imu.initialize(Bparameters);
        // End Init IMU
        prevHeading = getHeading();
        // Start Init IMU
        //makes sure we don't start with a weird val
        while (run = true) {
            //basically while the opmode is active, this will be getting the delta of heading
            double heading = 1;
            double deltaHeading = heading - prevHeading;
            degSec = deltaHeading / loopTime;
            try {
                sleep(20);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            //try to sleep for loopTime, else, print stacktrace.
            prevHeading = heading;
            //store current heading for use next loop
            opmode.telemetry.addData("degSec", getDegSec());
            opmode.telemetry.addData("loops", loops++);
        }
    }

    public void end() {
        run = false;
    }

    public double getDegSec() {
        return degSec;
    }
    public double getHeading(){
        //returns the Z axis (which is what you want if the Rev module is flat), for ease of use

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

}