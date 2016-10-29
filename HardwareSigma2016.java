package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.FileNotFoundException;
import java.text.DateFormat;
import java.util.Date;
import java.util.logging.Logger;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
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

    public static final double MID_SERVO       =  0 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    // Log utilities
    public static LoggerSigma2016 fileLogger = null;

    /* Constructor */
    public HardwareSigma2016(){

        // create the log file in phone storage
        if (fileLogger == null) {
            try {
                fileLogger = new LoggerSigma2016("ctu_log.txt");
                String currentDateTimeString = DateFormat.getDateTimeInstance().format(new Date());

                fileLogger.logLine("-- log@" + currentDateTimeString + "--");

            } catch (FileNotFoundException e) {
                e.printStackTrace();
            }
        }
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

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
//        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        pusherL = hwMap.servo.get("pusher_l");
        pusherR = hwMap.servo.get("pusher_r");
        pusherL.setPosition(MID_SERVO);
        pusherR.setPosition(MID_SERVO);

        //lineLightSensor = hwMap.lightSensor.get("line_light");
        beaconColorSensor = hwMap.colorSensor.get("beacon_color");

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

