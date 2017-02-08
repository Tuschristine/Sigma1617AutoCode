package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

import static org.firstinspires.ftc.teamcode.RedNearAutoOpSigma2016.GROUND_BRIGHTNESS_AVERAGE;

public class ReadSensorsSigma2016 extends Thread {

    boolean bReadCenterLightSensor = false;
    HardwareSigma2016 robot = null;


    ReadSensorsSigma2016(HardwareSigma2016 ourHardware) {
        robot = ourHardware;
        this.setPriority(this.MAX_PRIORITY);
    }

    public void SetCenterLightSensorRead(boolean onOff) {
        bReadCenterLightSensor = onOff;

        robot.centerLightSensorLevelMax = 0;
        robot.centerLightSensorLevel = 0;

        System.out.println("Sigma2016 -- center light sensor read is set to " + onOff);
    }

    public void run() {
        int lightlevelR, lightlevelG, lightlevelB;
        long curTime, timeInterval;
        long previousRunTime = 0;
        int expectedRunInterval = 5; // millisecond
        double speedTime1;
        double speedTime2;
        int position1;
        int position2;
        int angle1;
        int angle2;
        double currentSpeed;
        double currentAngleSpeed;

        while (true) {

            curTime = System.currentTimeMillis();
            if (previousRunTime != 0)
            {
                timeInterval = curTime - previousRunTime;

                if (timeInterval > expectedRunInterval*2)
                {
                    System.out.println("Sigma2016 -- sensor reading thread did not run for " + timeInterval + "ms");
                }
            }
            previousRunTime = curTime;

            if (bReadCenterLightSensor)
            {
                lightlevelB = robot.lineLightSensor.blue();
                lightlevelR = robot.lineLightSensor.red();
                lightlevelG = robot.lineLightSensor.green();

                robot.centerLightSensorLevel = lightlevelB + lightlevelR + lightlevelG;
                if (robot.centerLightSensorLevel > robot.centerLightSensorLevelMax)
                {
                    robot.centerLightSensorLevelMax = robot.centerLightSensorLevel;
                }
            }

            speedTime1 = System.currentTimeMillis();
            position1 = robot.frontLeftMotor.getCurrentPosition();
            speedTime2 = System.currentTimeMillis();
            position2 = robot.frontLeftMotor.getCurrentPosition();

            robot.currentSpeed = (Math.abs(position2 - position1)/(speedTime2 - speedTime1))/104.299 * 1000;

            speedTime1 = System.currentTimeMillis();
            angle1 = robot.gyro.getHeading();
            speedTime2 = System.currentTimeMillis();
            angle2 = robot.gyro.getHeading();

            robot.currentAngleSpeed = Math.abs(angle2 - angle1)/(speedTime2 - speedTime1) * 1000;

//            System.out.println("Sigma2016 -- Sensor reading thread is running. Priority = " + this.getPriority());
//            System.out.println("ReadSensors thread cur time = " + System.currentTimeMillis());

            try {
                sleep(expectedRunInterval);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
