package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.TypeConversion;

@I2cSensor(name = "MaxSonar I2CXL v2", description = "MaxSonar I2CXL Sensor from MaxBotix", xmlTag = "MaxSonarI2CXLv2")
public class I2CXLv2 extends I2cDeviceSynchDevice<I2cDeviceSynch>
{
    int lastDistance = -1;
    long lastPingTime;
    boolean waitingForNextPing = true;

    @Override
    public Manufacturer getManufacturer(){
        return Manufacturer.Other;
    }

    @Override
    protected synchronized boolean doInitialize(){
        return true;
    }

    @Override
    public String getDeviceName(){
        return "MaxSonarI2CXLv2";
    }

    public I2CXLv2(I2cDeviceSynch deviceClient){
        super(deviceClient, true);
        this.deviceClient.setI2cAddress(I2cAddr.create8bit(0xE0));
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    protected I2CXLv2(I2cDeviceSynch i2cDeviceSynch, boolean deviceClientIsOwned){
        super(i2cDeviceSynch, deviceClientIsOwned);
    }

    public void setI2cAddress(I2cAddr i2cAddr){
        deviceClient.setI2cAddress(i2cAddr);
    }

    private void ping(){
        deviceClient.write8(0, 0x51, I2cWaitControl.WRITTEN);
    }

    public int getDistance(){
        long curTime = System.currentTimeMillis();
        if(((curTime - lastPingTime) > 80) && !waitingForNextPing){
            int potentialDistance = TypeConversion.byteArrayToShort(deviceClient.read(0x01, 2));
            lastDistance = potentialDistance;
            waitingForNextPing = true;
        } if((System.currentTimeMillis() - lastPingTime) > 100){
            ping();
            lastPingTime = System.currentTimeMillis();
            waitingForNextPing = false;
        }
        return lastDistance;
    }
}