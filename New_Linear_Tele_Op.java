package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by sidharth on 9/28/16.
 */


//THE LINEAR TELE OP FOR SIGMAS 2016 with drive and beacon pushers.

@TeleOp(name="Linear Tele Op New", group="Linear Opmode")

public class New_Linear_Tele_Op extends LinearOpMode{

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    Servo beaconL;
    Servo beaconR;

    float leftThrottle;
    float rightThrottle;
    boolean y;
    boolean a;
    boolean up;
    boolean down;
    double beaconLPos;
    double beaconRPos;
    int increment;

    HardwareSigma2016 robot = new HardwareSigma2016();   // Use a Pushbot's hardware

    @Override
    public void runOpMode() {
        myinit();
        increment = 5;
        waitForStart();

        while (opModeIsActive()) {

            leftThrottle = gamepad1.left_stick_y;
            rightThrottle = gamepad1.right_stick_y;

            a = gamepad2.a;
            y = gamepad2.y;
            up = gamepad2.dpad_up;
            down = gamepad2.dpad_down;

            float right = -rightThrottle;
            float left = -leftThrottle;

            right = (float) scaleInput(right);
            left = (float) scaleInput(left);

            if (increment == 11){
                frontLeft.setPower(left);
                backLeft.setPower(left);
                frontRight.setPower(-right);
                backRight.setPower(-right);
                telemetry.addData("increment:", increment);
                telemetry.update();
            }

            if (increment == 10){
                frontLeft.setPower(left);
                backLeft.setPower(left);
                frontRight.setPower(-right);
                backRight.setPower(-right);
                telemetry.addData("increment:", increment);
                telemetry.update();
            }

            if (increment == 9){
                frontLeft.setPower(left * 0.9);
                backLeft.setPower(left * 0.9);
                frontRight.setPower(-right * 0.9);
                backRight.setPower(-right * 0.9);
                telemetry.addData("increment:", increment);
                telemetry.update();
            }

            if (increment == 8){
                frontLeft.setPower(left * 0.8);
                backLeft.setPower(left * 0.8);
                frontRight.setPower(-right * 0.8);
                backRight.setPower(-right * 0.8);
                telemetry.addData("increment:", increment);
                telemetry.update();
            }

            if (increment == 7){
                frontLeft.setPower(left * 0.7);
                backLeft.setPower(left * 0.7);
                frontRight.setPower(-right * 0.7);
                backRight.setPower(-right * 0.7);
                telemetry.addData("increment:", increment);
                telemetry.update();
            }

            if (increment == 6){
                frontLeft.setPower(left * 0.6);
                backLeft.setPower(left * 0.6);
                frontRight.setPower(-right * 0.6);
                backRight.setPower(-right * 0.6);
                telemetry.addData("increment:", increment);
                telemetry.update();
            }

            if (increment == 5){
                frontLeft.setPower(left * 0.5);
                backLeft.setPower(left * 0.5);
                frontRight.setPower(-right * 0.5);
                backRight.setPower(-right * 0.5);
                telemetry.addData("increment:", increment);
                telemetry.update();
            }

            if (increment == 4){
                frontLeft.setPower(left * 0.4);
                backLeft.setPower(left * 0.4);
                frontRight.setPower(-right * 0.4);
                backRight.setPower(-right * 0.4);
                telemetry.addData("increment:", increment);
                telemetry.update();
            }

            if (increment == 3){
                frontLeft.setPower(left * 0.3);
                backLeft.setPower(left * 0.3);
                frontRight.setPower(-right * 0.3);
                backRight.setPower(-right * 0.3);
                telemetry.addData("increment:", increment);
                telemetry.update();
            }

            if (increment == 2){
                frontLeft.setPower(left * 0.3);
                backLeft.setPower(left * 0.3);
                frontRight.setPower(-right * 0.3);
                backRight.setPower(-right * 0.3);
                telemetry.addData("increment:", increment);
                telemetry.update();
            }

            /**
             * The tolerance for the variable
             */

            if (increment < 3){
                increment = 3;
            }

            if (increment > 10){
                increment = 10;
            }

            if(gamepad1.right_trigger == 1) {
                increment -= 1;
                try {
                    Thread.sleep(200);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            if (gamepad1.left_trigger == 1){
                increment += 1;
                try {
                    Thread.sleep(200);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            if (gamepad1.left_bumper){
                increment  = 10;
            }

            if (gamepad1.right_bumper){
                increment  = 5;
            }



            if(y){
                beaconR.setPosition(1.0);
                try {
                    Thread.sleep(1300);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                beaconR.setPosition(-1.0);
                try {
                    Thread.sleep(1300);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                beaconR.setPosition(0.5);
            }
            else{
                beaconR.setPosition(0.5);

            }

            if(up){
                beaconL.setPosition(-1.0);
                try {
                    Thread.sleep(1300);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                beaconL.setPosition(1.0);
                try {
                    Thread.sleep(1300);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                beaconL.setPosition(0.5);
            }
            else{
                beaconL.setPosition(0.5);
            }


            /*if(beaconRPos > 0.9){
                beaconRPos = 0.9;
            }
            else if(beaconRPos < 0.2){
                beaconRPos = 0.2;
            }
            if(beaconLPos > 0.9){
                beaconLPos = 0.9;
            }
            else if(beaconLPos < 0.2){
                beaconLPos = 0.2;
            }*/

            //beaconLPos = Range.clip(beaconLPos, -1.0, 1.0);
            //beaconRPos = Range.clip(beaconRPos, -1.0, 1.0);
        }
    }
    public void myinit(){
        frontRight = hardwareMap.dcMotor.get("motor_3");
        frontLeft = hardwareMap.dcMotor.get("motor_1");
        backRight = hardwareMap.dcMotor.get("motor_4");
        backLeft = hardwareMap.dcMotor.get("motor_2");
        beaconL = hardwareMap.servo.get("pusher_l");
        beaconR = hardwareMap.servo.get("pusher_r");

        //beaconL.scaleRange(0.2, 0.9);
        //beaconR.scaleRange(0.2, 0.9);



        //beaconL.setPosition(1.0);
        //beaconR.setPosition(0.0);
    }
    double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.1, 0.12, 0.15, 0.18, 0.24, 0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        int index = (int) (dVal * 16.0);
        if(index < 0){
            index = -index;
        } else if(index > 16){
            index = 16;
        }

        double dScale = 0.0;
        if(dVal < 0){
            dScale = -scaleArray[index];
        } else{
            dScale = scaleArray[index];
        }

        return dScale;
    }
}
