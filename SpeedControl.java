package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;

/**
 * Created by Christine on 1/28/17.
 */
@Autonomous(name = "speedControl", group = "Sigma6710")

public class SpeedControl extends LinearOpMode {

    static final double COUNTS_PER_MOTOR_REV = 1310 * 1.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1 / 1.6;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.9;     // Nominal speed for better accuracy.
    static final double P_DRIVE_COEFF = 0.05;     // Larger is more responsive, but also less stable

    static final double TURN_SPEED = 0.25;     // Nominal half speed for better accuracy.
    static final double TURN_THRESHOLD = 2;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.05;     // Larger is more responsive, but also less stable
    static final double MIN_TURN_SPEED = 0.15;     // Larger is more responsive, but also less stable with over turn

    static final double maxLeftRightSpeedDifferentialAtDrive = 0.5;

    static final double WALL_APPROACHING_SPEED = 0.6;
    static final double P_WALL_APPROACHING_COEFF = 0.05;

    static final double LINE_DETECTION_SPEED = 0.05;
    static final double WALL_TRAVELING_SPEED = 0.5;
    static final double P_WALL_TRACKING_COEFF_FINE = 0.025;// Larger is more responsive, but also less stable
    static final double P_WALL_TRACKING_COEFF_COARSE = 0.05;// Larger is more responsive, but also less stable

    static final double TARGET_WALL_DISTANCE_FORWARD = 7;  // ultrasound sensor reading for x inch away from wall
    static final double TARGET_WALL_DISTANCE_BACKWARD = 8;
    static final double WALL_DISTANCE_THRESHOLD = 0.5; // no need to adjust if wall distance is within range

    static final int ENCODER_TARGET_THRESHOLD = 10;

    static final int RED_TRESHOLD = 5;
    static final int BLUE_TRESHOLD = 5;

    static final double AngleThresh = 2;
    static final double SpeedThresh = 0.5;
    static final double PowerIncrement = 0.1;
    HardwareSigma2016 robot = null;

    //@Override
    public void runOpMode() throws InterruptedException {
        robot = new HardwareSigma2016();
        robot.init(hardwareMap);
    }

    public void gyroDrive(double power,
        double distance,
        double angle,
        double speed){


            int newLeftTarget;
            int newRightTarget;
            int moveCounts;
            double max;
            double error;
            double steer;
            double leftSpeed;
            double rightSpeed;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // reset encoder
                robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                // set mode
                robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backLeftMotor.setMode(RUN_WITHOUT_ENCODER);
                robot.backRightMotor.setMode(RUN_WITHOUT_ENCODER);

                // Determine new target position, and pass to motor controller
                moveCounts = (int) (distance * COUNTS_PER_INCH);
                newLeftTarget = robot.frontLeftMotor.getCurrentPosition() + moveCounts;
                newRightTarget = robot.frontRightMotor.getCurrentPosition() + moveCounts;

                // Set Target and Turn On RUN_TO_POSITION
                robot.frontLeftMotor.setTargetPosition(newLeftTarget);
                robot.frontRightMotor.setTargetPosition(newRightTarget);

                robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // determine back motor's direction
                if (distance < 0) {
                    if (robot.backLeftMotor.getDirection() == DcMotorSimple.Direction.FORWARD) {
                        robot.backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    } else {
                        robot.backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                    }

                    if (robot.backRightMotor.getDirection() == DcMotorSimple.Direction.FORWARD) {
                        robot.backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    } else {
                        robot.backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                    }
                }

                // start motion.
                speed = Range.clip(Math.abs(speed), 0.0, 1.0);
                // keep looping while we are still active, and BOTH motors are running.
                while (opModeIsActive() &&
                        (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy())) {

                    if (Math.abs(robot.frontLeftMotor.getCurrentPosition()
                            - robot.frontLeftMotor.getTargetPosition()) <= ENCODER_TARGET_THRESHOLD) {
                        break;
                    } else if (Math.abs(robot.frontRightMotor.getCurrentPosition()
                            - robot.frontRightMotor.getTargetPosition()) <= ENCODER_TARGET_THRESHOLD) {
                        break;
                    }

                    if (speed - robot.currentSpeed > SpeedThresh) {
                        power += PowerIncrement;
                    } else if (speed - robot.currentSpeed < -SpeedThresh) {
                        power -= PowerIncrement;
                    }

                    speed = Range.clip(Math.abs(speed), 0.0, 1.0);
                    robot.frontLeftMotor.setPower(speed);
                    robot.frontRightMotor.setPower(speed);
                    robot.backRightMotor.setPower(speed);
                    robot.backLeftMotor.setPower(speed);

                    // adjust relative speed based on heading error.
                    error = getError(angle);
                    steer = getSteer(error, P_DRIVE_COEFF);

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (distance < 0)
                        steer *= -1.0;

                    leftSpeed = speed - steer;
                    rightSpeed = speed + steer;

                    leftSpeed = Range.clip(leftSpeed,
                            speed - Math.abs(maxLeftRightSpeedDifferentialAtDrive * speed),
                            speed + Math.abs(maxLeftRightSpeedDifferentialAtDrive * speed));
                    rightSpeed = Range.clip(rightSpeed,
                            speed - Math.abs(maxLeftRightSpeedDifferentialAtDrive * speed),
                            speed + Math.abs(maxLeftRightSpeedDifferentialAtDrive * speed));

                    robot.frontLeftMotor.setPower(leftSpeed);
                    robot.frontRightMotor.setPower(rightSpeed);
                    robot.backLeftMotor.setPower(leftSpeed);
                    robot.backRightMotor.setPower(rightSpeed);

                    // Display drive status for the driver.
                    telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                    telemetry.addData("Targets L:R", "%7d:%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Actual", "%7d:%7d", robot.frontLeftMotor.getCurrentPosition(),
                            robot.frontRightMotor.getCurrentPosition());

                    telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                    telemetry.update();
                }

                // Turn off RUN_TO_POSITION
                robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                if (distance < 0) {
                    if (robot.backLeftMotor.getDirection() == DcMotorSimple.Direction.FORWARD) {
                        robot.backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    } else {
                        robot.backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                    }

                    if (robot.backRightMotor.getDirection() == DcMotorSimple.Direction.FORWARD) {
                        robot.backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    } else {
                        robot.backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                    }
                }
            }
        }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.gyro.getIntegratedZValue();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void StopAllMotion() {
        // Stop all motion;
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.intake.setPower(0);
        robot.flicker.setPower(0);
    }

}