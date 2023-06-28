package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
@I2cSensor(name = "Husky Lens Camera", description = "Husky Lens Camera from DFRobot", xmlTag = "HuskyLens")

public class HuskyLens extends I2cDeviceSynchDevice<I2cDeviceSynch>
{
    @Override
    public Manufacturer getManufacturer()
    {

        return Manufacturer.Adafruit;
    }

    @Override
    protected synchronized boolean doInitialize()
    {
        return true;
    }

    @Override
    public String getDeviceName()
    {

        return "Husky Lens Camera";
    }

    public HuskyLens(I2cDeviceSynch deviceClient)
    {
        super(deviceClient, true);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();

    }
}