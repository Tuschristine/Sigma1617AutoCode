package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.BlueNearAutoOpSigma2016.fileLogger;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 */

public class HardwareSigma2016
{
    /* Public OpMode members. */
    public DcMotor  backLeftMotor = null;
    public DcMotor  backRightMotor = null;
    public DcMotor frontLeftMotor = null;
    public DcMotor  frontRightMotor = null;
    public Servo    pusherL    = null;
    public Servo    pusherR   = null;
    public LightSensor lineLightSensor = null;
    public ColorSensor beaconColorSensor = null;
    public UltrasonicSensor ultrasonicSensor = null;

    public static final double PUSHER_L_IN  =  1.0 ;
    public static final double PUSHER_R_IN  =  0.0 ;
    public static final double PUSHER_L_OUT  =  0.0 ;
    public static final double PUSHER_R_OUT  =  1.0 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareSigma2016(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeftMotor = hwMap.dcMotor.get("motor_2");
        frontRightMotor  = hwMap.dcMotor.get("motor_3");
        backLeftMotor  = hwMap.dcMotor.get("motor_1");
        backRightMotor = hwMap.dcMotor.get("motor_4");
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);

        // Define and initialize ALL installed servos.
        pusherL = hwMap.servo.get("pusher_l");
        pusherR = hwMap.servo.get("pusher_r");
        pusherL.setPosition(PUSHER_L_IN);
        pusherR.setPosition(PUSHER_R_IN);

        // light sensor on the robot bottom
//        lineLightSensor = hwMap.lightSensor.get("line_light");

        // color sensor on beacon pusher
        beaconColorSensor = hwMap.colorSensor.get("beacon_color");
//        beaconColorSensor.enableLed(true);
//        try {
//            Thread.sleep(300);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
        beaconColorSensor.enableLed(false);

        // ultrasonic sensor
        ultrasonicSensor = hwMap.ultrasonicSensor.get("ultrasonic");

        fileLogger.logLine("Hardware is initialized.");
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

