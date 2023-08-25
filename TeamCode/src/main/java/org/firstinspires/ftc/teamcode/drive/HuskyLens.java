package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.TypeConversion;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

@I2cSensor(name = "Husky Lens Camera", description = "Husky Lens Camera from DFRobot", xmlTag = "HuskyLens")

public class HuskyLens extends I2cDeviceSynchDevice<I2cDeviceSynch>
{
    String commandHeaderAndAddress = "55AA11";
    HashMap<String, String> algorthimsByteID = new HashMap<>();
    private boolean checkOnceAgain = true;

    public HuskyLens(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);
        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
        algorthimsByteID.put("ALGORITHM_OBJECT_TRACKING", "0100");
        algorthimsByteID.put("ALGORITHM_FACE_RECOGNITION", "0000");
        algorthimsByteID.put("ALGORITHM_OBJECT_RECOGNITION", "0200");
        algorthimsByteID.put("ALGORITHM_LINE_TRACKING", "0300");
        algorthimsByteID.put("ALGORITHM_COLOR_RECOGNITION", "0400");
        algorthimsByteID.put("ALGORITHM_TAG_RECOGNITION", "0500");
        algorthimsByteID.put("ALGORITHM_OBJECT_CLASSIFICATION", "0600");
        algorthimsByteID.put("ALGORITHM_QR_CODE_RECOGNTITION", "0700");
        algorthimsByteID.put("ALGORITHM_BARCODE_RECOGNTITION", "0800");
    }

    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.Unknown;
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
    //public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x11);
    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x32);

    protected void setOptimalReadWindow()
    {
        // Sensor registers are read repeatedly and stored in a register. This method specifies the
        // registers and repeat read mode
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.FIRST.bVal,
                Register.LAST.bVal - Register.FIRST.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }
    
    public enum Register
    {
        FIRST(0),
        HEADER(0x55),
        HEADER_2(0xAA),
        ADDRESS(0x11),
        DATA_LENGTH(0x0A),
        COMMAND(0X2A),
        LAST(COMMAND.bVal);

        public int bVal;

        Register(int bVal)
        {
            this.bVal = bVal;
        }
    }

    protected void writeShort(final Register reg, short value)
    {
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }

    protected short readShort(Register reg)
    {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }

    public Object knock() {
        byte[] cmd = cmdToBytes(commandHeaderAndAddress + "002c3c");
        deviceClient.write(cmd);
        return processReturnData();
    }

    public String knock2() {
        byte[] bytesToSend = new byte[] {0x55, (byte)0xAA, 0x11, 0x00, 0x2C, 0x3C};
        deviceClient.write(bytesToSend);
        byte[] bytesReceived = deviceClient.read(6);
        byte[] expectedBytes = new byte[] {0x55, (byte)0xAA, 0x11, 0x00, 0x2E, 0x3E};
        for (int i = 0; i < 6; i++) {
            if (bytesReceived[i] != expectedBytes[i]) return "Knock2 Return Didn't Match";
        }
        return "Knock2 Received";
    }

    public byte[] cmdToBytes(String cmd) {
        int length = cmd.length();
        byte[] bytes = new byte[length / 2];
        for (int i = 0; i < length; i += 2) {
            bytes[i / 2] = (byte) Integer.parseInt(cmd.substring(i, i + 2), 16);
        }
        return bytes;
    }

//    public List<Object> processReturnData(boolean numIdLearnFlag, boolean frameFlag) {
      public String processReturnData(/*boolean numIdLearnFlag, boolean frameFlag*/) {
        String byteString = "";
        try {
            byte[] responseBytes = deviceClient.read(5);
            for (int i = 0; i < 5; i++) {
                //byteString += String.valueOf((char) huskylensSer.read_byte(address));
                byteString += String.valueOf((char) responseBytes[i]);
            }
            // NOTE: The start register might be 6 instead of 5. Don't know if registers are zero based.
            responseBytes = deviceClient.read(5, byteString.charAt(3) + 1);
            for (int i = 0; i < (byteString.charAt(3) + 1); i++) {
                //byteString += String.valueOf((char) huskylensSer.read_byte(address));
                byteString += String.valueOf((char) responseBytes[i]);
            }
            String[] commandSplit = splitCommandToParts(byteString);
            if (commandSplit[3].equals("2e")) {
                checkOnceAgain = true;
                return "Knock Received";
            }
            else {
                return "Knock Not Received";
            }
//            } else {
//                List<Object> returnData = new ArrayList<>();
//                int numberOfBlocksOrArrow = Integer.parseInt(commandSplit[4].substring(2, 4) + commandSplit[4].substring(0, 2), 16);
//                int numberOfIDLearned = Integer.parseInt(commandSplit[4].substring(6, 8) + commandSplit[4].substring(4, 6), 16);
//                int frameNumber = Integer.parseInt(commandSplit[4].substring(10, 12) + commandSplit[4].substring(8, 10), 16);
//                boolean isBlock = true;
//                for (int i = 0; i < numberOfBlocksOrArrow; i++) {
//                    Object[] tmpObj = getBlockOrArrowCommand();
//                    isBlock = (boolean) tmpObj[1];
//                    returnData.add(tmpObj[0]);
//                }
//
//                List<Object> finalData = new ArrayList<>();
//                List<Object> tmp = new ArrayList<>();
//                for (Object i : returnData) {
//                    tmp = new ArrayList<>();
//                    for (int q = 0; q < i.length(); q += 4) {
//                        int low = Integer.parseInt(i.substring(q, q + 2), 16);
//                        int high = Integer.parseInt(i.substring(q + 2, q + 4), 16);
//                        int val = (high > 0) ? low + 255 + high : low;
//                        tmp.add(val);
//                    }
//                    finalData.add(tmp);
//                    tmp = new ArrayList<>();
//                }
//                checkOnceAgain = true;
//                List<Object> ret = convertToClassObject(finalData, isBlock);
//                if (numIdLearnFlag) {
//                    ret.add(numberOfIDLearned);
//                }
//                if (frameFlag) {
//                    ret.add(frameNumber);
//                }
//                return ret;
//            }
        } catch (Exception e) {
            if (checkOnceAgain) {
                //huskylensSer.timeout = 5;
                checkOnceAgain = false;
                //huskylensSer.timeout = 0.5;
                return processReturnData();
            }
            System.out.println("Read response error, please try again");
            //huskylensSer.flushInput();
            //huskylensSer.flushOutput();
            //huskylensSer.flush();
            return "Error";
        }
    }

    public String[] splitCommandToParts(String str) {
        String headers = str.substring(0, 4);
        String address = str.substring(4, 6);
        int dataLength = Integer.parseInt(str.substring(6, 8), 16);
        String command = str.substring(8, 10);
        String data = "";
        if (dataLength > 0) {
            data = str.substring(10, 10 + dataLength * 2);
        }
        String checkSum = str.substring(2 * (6 + dataLength - 1), 2 * (6 + dataLength - 1) + 2);
        return new String[]{headers, address, Integer.toString(dataLength), command, data, checkSum};
    }
}