/*
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.io.FileNotFoundException;
import java.text.DateFormat;
import java.util.Date;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static org.firstinspires.ftc.teamcode.HardwareSigma2016.PUSHER_L_IN;
import static org.firstinspires.ftc.teamcode.HardwareSigma2016.PUSHER_L_OUT;
import static org.firstinspires.ftc.teamcode.HardwareSigma2016.PUSHER_R_IN;
import static org.firstinspires.ftc.teamcode.HardwareSigma2016.PUSHER_R_OUT;

/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 * <p>
 * The code REQUIRES that you DO have encoders on the wheels,
 * otherwise you would use: PushbotAutoDriveByTime;
 * <p>
 * This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 * otherwise you would use: PushbotAutoDriveByEncoder;
 * <p>
 * This code requires that the drive Motors have been configured such that a positive
 * power command moves them forward, and causes the encoders to count UP.
 * <p>
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 * <p>
 * In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
 * This is performed when the INIT button is pressed on the Driver Station.
 * This code assumes that the robot is stationary when the INIT button is pressed.
 * If this is not the case, then the INIT should be performed again.
 * <p>
 * Note: in this example, all angles are referenced to the initial coordinate frame set during the
 * the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 * <p>
 * The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 * which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 * This is consistent with the FTC field coordinate conventions set out in the document:
 * ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "Blue Near Auto Op Sigma 2016", group = "Sigma6710")
//@Disabled
public class BlueNearAutoOpSigma2016 extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareSigma2016 robot = new HardwareSigma2016();   // Use a Pushbot's hardware
    ModernRoboticsI2cGyro gyro = null;                    // Additional Gyro device

    static final double COUNTS_PER_MOTOR_REV = 2250;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 0.666666667;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 5.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.8;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.6;     // Nominal half speed for better accuracy.
    static final double WALL_APPROACHING_SPEED = 0.3;
    static final double WALL_TRACKING_SPEED = 0.06;

    static final double HEADING_THRESHOLD = 3;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.5;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable
    static final double P_WALL_TRACKING_COEFF = 0.1;     // Larger is more responsive, but also less stable

    static final double TARGET_WALL_DISTANCE = 12.0;  // ultrasound sensor reading for x inch away from wall
    static final double WALL_DISTANCE_THRESHOLD = 1.0; // no need to adjust if wall distance is within range
    static final double WALL_TRACKING_MAX_HEADING_OFFSET = 3;

    static final double COLOR_COEFF = 0.3;

    static final int RED_TRESHOLD = 2500;
    static final int BLUE_TRESHOLD = 10000;
    static final int WHITE_LINE_THRESHOLD = 400;

    // Logging utilities
    public static LoggerSigma2016 fileLogger = null;

    int ct2 = 0;
    int ct1 = 0;
    int ct3 = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        // create the log file in phone storage
        if (fileLogger == null) {
            try {
                fileLogger = new LoggerSigma2016("ctu_log.txt");
                String currentDateTimeString = DateFormat.getDateTimeInstance().format(new Date());

                fileLogger.logLine("--BlueNear log@" + currentDateTimeString + "--");

            } catch (FileNotFoundException e) {
                e.printStackTrace();
            }
        }

        /* -------- initializations ---------- */
        /*
        * Initialize the standard drive system variables.
        * The init() method of the hardware class does most of the work here
        */
        robot.init(hardwareMap);

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro.isCalibrating()) {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            telemetry.update();
            idle();
        }

        gyro.resetZAxisIntegrator();

        /* -------- driving to the predefined position ------- */
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
        fileLogger.logLine("0 -- gyro reading=" + gyro.getIntegratedZValue());

        gyroDrive(DRIVE_SPEED, -18.0, 0.0); // Drive BWD 30 inches
        fileLogger.logLine("1 -- gyro reading=" + gyro.getIntegratedZValue());

        gyroTurn(TURN_SPEED, -60.0);               // Turn to -60 Degrees
        fileLogger.logLine("2 -- gyro reading=" + gyro.getIntegratedZValue());

        gyroDrive(DRIVE_SPEED, -48, -60.0); // Drive BWD 49 inches
        fileLogger.logLine("3 -- gyro reading=" + gyro.getIntegratedZValue());

        gyroTurn(TURN_SPEED, -30.0);               // Turn to -10 degree
        fileLogger.logLine("4 -- gyro reading=" + gyro.getIntegratedZValue());

        UltraSonicReachTheWall(WALL_APPROACHING_SPEED, -80, -10.0);

        gyroTurn(TURN_SPEED, 0.0);               // Turn to 0 degree
        fileLogger.logLine("5 -- gyro reading=" + gyro.getIntegratedZValue());

        telemetry.addData("Initial Path", "Complete");
        telemetry.update();

        /* ------ ultrasonic wall tracker + white line detection ------- */
//        double distanceFromWall;
//        ElapsedTime holdTimer = new ElapsedTime();
//        double holdTime = 100; //ten second time out

        // keep looping while we have time remaining.
//        holdTimer.reset();
//        while (holdTimer.time() <= holdTime) {
//            distanceFromWall = robot.ultrasonicSensor.getUltrasonicLevel();
//            telemetry.addData("UltraSound level: ", "%d", distanceFromWall);
//            telemetry.update();
//
//            sleep(100);
//            idle();
//        }

        // Drive forward to align with the wall and park at far line
        WallTrackingToWhiteLine(WALL_TRACKING_SPEED, -80, 0, true);
//        WallTrackingToColorBeacon(WALL_TRACKING_SPEED, -60, 0.0, true);

//        // Align up with the beacon lights
//        gyroDrive(DRIVE_SPEED, 1.0, 0.0); // Drive FWD 1 inches

        // run the beacon light color detection and button pushing procedure
        ColorDetectionAndButtonPushing();

        // Drive forward to detect the near line
//        WallTrackingToWhiteLine(0.5, -72, true);
//
//        WallTrackingToColorBeacon(WALL_TRACKING_SPEED * 3, 53, 0.0, false);
//        WallTrackingToColorBeacon(WALL_TRACKING_SPEED, -18, 0.0, true);

        WallTrackingToWhiteLine(WALL_TRACKING_SPEED, 60.0, 0, true);



        // Align up with the beacon lights
//        gyroDrive(DRIVE_SPEED, 1.0, 0.0); // Drive BWD 1 inches
//
        // run the beacon light color detection and button pushing procedure
        ColorDetectionAndButtonPushing();

        /*------ drive back to the vortex ------*/
        /*
        // Drive forward to align with the wall, no white line detection
        WallTrackingToWhiteLine(0.8, 72, false);

        gyroDrive(DRIVE_SPEED, 15.00, -45.0); // Drive BWD 50 inches heading -45 degree
        fileLogger.logLine("5 -- gyro reading=" + gyro.getIntegratedZValue());

        gyroTurn(TURN_SPEED, 90.0);         // Turn  CCW to -45 Degrees
        fileLogger.logLine("6 -- gyro reading=" + gyro.getIntegratedZValue());

        gyroDrive(DRIVE_SPEED, 15.00, -155.0); // Drive BWD 30 inches heading 45 degree
        fileLogger.logLine("7 -- gyro reading=" + gyro.getIntegratedZValue());
*/

        // All work are finished. Close the log file.
        fileLogger.close();
    }

    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive(double speed,
                          double distance,
                          double angle) {

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
            robot.backRightMotor.setMode((RUN_WITHOUT_ENCODER));

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newLeftTarget = robot.frontLeftMotor.getCurrentPosition() + moveCounts;
            newRightTarget = robot.frontRightMotor.getCurrentPosition() + moveCounts;
//            newLeftTarget = robot.backLeftMotor.getCurrentPosition() + moveCounts;
//            newRightTarget = robot.backRightMotor.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.frontLeftMotor.setTargetPosition(newLeftTarget);
            robot.frontRightMotor.setTargetPosition(newRightTarget);
//            robot.backLeftMotor.setTargetPosition(newLeftTarget);
//            robot.backRightMotor.setTargetPosition(newRightTarget);

            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);  NOT SURE CHANGE LATER MAYBE

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
            robot.frontLeftMotor.setPower(speed);
            robot.frontRightMotor.setPower(speed);
            robot.backRightMotor.setPower(speed);
            robot.backLeftMotor.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.frontLeftMotor.setPower(leftSpeed);
                robot.frontRightMotor.setPower(rightSpeed);
                robot.backLeftMotor.setPower(leftSpeed);
                robot.backRightMotor.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Actual", "%7d:%7d", robot.frontLeftMotor.getCurrentPosition(),
                        robot.frontRightMotor.getCurrentPosition());

                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.frontLeftMotor.setPower(0);
            robot.frontRightMotor.setPower(0);
            robot.backLeftMotor.setPower(0);
            robot.backRightMotor.setPower(0);

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

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        robot.frontLeftMotor.setMode(RUN_WITHOUT_ENCODER);
        robot.frontRightMotor.setMode(RUN_WITHOUT_ENCODER);
        robot.backLeftMotor.setMode(RUN_WITHOUT_ENCODER);
        robot.backRightMotor.setMode((RUN_WITHOUT_ENCODER));

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.frontLeftMotor.setPower(leftSpeed);
        robot.frontRightMotor.setPower(rightSpeed);
        robot.backLeftMotor.setPower(leftSpeed);
        robot.backRightMotor.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
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

    public boolean UltraSonicReachTheWall(double speed,
                                          double distance,
                                          double angle) {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;
        double ultraSoundLevel;
        double blackLightLevel, lightLevel;

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
            robot.frontLeftMotor.setPower(speed);
            robot.frontRightMotor.setPower(speed);
            robot.backRightMotor.setPower(speed);
            robot.backLeftMotor.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.frontLeftMotor.setPower(leftSpeed);
                robot.frontRightMotor.setPower(rightSpeed);
                robot.backLeftMotor.setPower(leftSpeed);
                robot.backRightMotor.setPower(rightSpeed);

                ultraSoundLevel = robot.ultrasonicSensor.getUltrasonicLevel();

                // handles abnormal ultrasonic reading
                if (ultraSoundLevel == 0) {
                    // stop the robot
                    robot.frontLeftMotor.setPower(0);
                    robot.frontRightMotor.setPower(0);
                    robot.backLeftMotor.setPower(0);
                    robot.backRightMotor.setPower(0);

                    fileLogger.logLine("abnormal -- ulevel=" + ultraSoundLevel);

                    sleep(100);
                    idle();
                } else if (ultraSoundLevel <= TARGET_WALL_DISTANCE) {

                    fileLogger.logLine("wall reached -- ulevel=" + ultraSoundLevel);
                    // reached the wall. stop.
                    break;
                }

                ct2++;
                if (ct2 > 1000) {
                    ct2 = 0;
                    fileLogger.logLine("ultrasound level = " + ultraSoundLevel);
                }
            }

            // Stop all motion;
            robot.frontLeftMotor.setPower(0);
            robot.frontRightMotor.setPower(0);
            robot.backLeftMotor.setPower(0);
            robot.backRightMotor.setPower(0);

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

        return (true);
    }


    /**
     * Method to track along a wall using an ultrasonic sensor
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired distance (timeout if no white line found)
     * 2) Driver stops the opmode running.
     * 3) White line on the ground is detected and aligned by the light sensors
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     */
    public void WallTrackingToWhiteLine(double speed,
                                        double distance,
                                        double headingAngle,
                                        boolean bLineDetection) {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer = 0;
        double leftSpeed;
        double rightSpeed;
        double ultraSoundLevel, angleOffset;
        int lightlevelR, lightlevelB, lightlevelG;
        int lightlevel = 0;
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
            robot.frontLeftMotor.setPower(speed);
            robot.frontRightMotor.setPower(speed);
            robot.backRightMotor.setPower(speed);
            robot.backLeftMotor.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() )
//                    &&
//                    (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy())) {
            {
                // adjust relative speed based on ultrasound reading.
                ultraSoundLevel = robot.ultrasonicSensor.getUltrasonicLevel();
                error = ultraSoundLevel - TARGET_WALL_DISTANCE;

                angleOffset = gyro.getIntegratedZValue() - headingAngle;

                ct3++;
                if (ct3 > 1000) {
                    ct3 = 0;

                    fileLogger.logLine("ultrasoniclevel=" + ultraSoundLevel + " error=" + error + " angleOffset=" + angleOffset);
                }

                if ((Math.abs(error) >= WALL_DISTANCE_THRESHOLD) &&
                        ((Math.abs(angleOffset) < WALL_TRACKING_MAX_HEADING_OFFSET) || (error * angleOffset * distance < 0))) {

                    steer = getSteer(error, P_WALL_TRACKING_COEFF);

                    leftSpeed = speed - steer;
                    rightSpeed = speed + steer;

                    // Normalize speeds if any one exceeds +/- 1.0;
                    max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                    if (max > 1.0) {
                        leftSpeed /= max;
                        rightSpeed /= max;
                    }

                    robot.frontLeftMotor.setPower(leftSpeed);
                    robot.frontRightMotor.setPower(rightSpeed);
                    robot.backLeftMotor.setPower(leftSpeed);
                    robot.backRightMotor.setPower(rightSpeed);

                    if (ct3 == 0) {
                        fileLogger.logLine("error=" + error
                                + " leftspeed=" + String.format(Double.toString(leftSpeed), "%5.2f")
                                + " rightSpeed=" + String.format(Double.toString(rightSpeed), "%5.2f"));
                    }
                } else {
                    robot.frontLeftMotor.setPower(speed);
                    robot.frontRightMotor.setPower(speed);
                    robot.backRightMotor.setPower(speed);
                    robot.backLeftMotor.setPower(speed);
                }

                if (bLineDetection) {

                    lightlevelR = robot.lineLightSensor.blue();
                    lightlevelB = robot.lineLightSensor.red();
                    lightlevelG = robot.lineLightSensor.green();
                    lightlevel = lightlevelB + lightlevelR + lightlevelG;

                    fileLogger.logLine("Light Level " + robot.groundbrightness + " " + lightlevel);
                    telemetry.addData("Light Level :: ", "%d,%d", robot.groundbrightness, lightlevel);
                    telemetry.update();

                    if(lightlevel > 2.5 * robot.groundbrightness){
                        //white line detected
                        break;
                    }
                }
            }

            // Stop all motion;
            robot.frontLeftMotor.setPower(0);
            robot.frontRightMotor.setPower(0);
            robot.backLeftMotor.setPower(0);
            robot.backRightMotor.setPower(0);

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

    /**
     * Method to track along a wall using an ultrasonic sensor
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired distance (timeout if no white line found)
     * 2) Driver stops the opmode running.
     * 3) White line on the ground is detected and aligned by the light sensors
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     */
    public boolean WallTrackingToColorBeacon(double speed,
                                             double distance,
                                             double headingAngle,
                                             boolean bBeaconDetection) {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer = 0;
        double leftSpeed;
        double rightSpeed;
        double ultraSoundLevel, angleOffset;
        int red, green, blue;
        int passRedCounter = 0, passBlueCounter = 0;
        int redMax = 0, blueMax = 0;
        int redCheck = 0, blueCheck = 0;
        int lightPassed = 0;

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

                lightPassed = 1;
            }

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.frontLeftMotor.setPower(speed);
            robot.frontRightMotor.setPower(speed);
            robot.backRightMotor.setPower(speed);
            robot.backLeftMotor.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy())) {

                // adjust relative speed based on ultrasound reading.
                ultraSoundLevel = robot.ultrasonicSensor.getUltrasonicLevel();
                error = ultraSoundLevel - TARGET_WALL_DISTANCE;

                angleOffset = gyro.getIntegratedZValue() - headingAngle;

                ct3++;
                if (ct3 > 1000) {
                    ct3 = 0;

                    fileLogger.logLine("ultrasoniclevel=" + ultraSoundLevel + " error=" + error + " angleOffset=" + angleOffset);
                }

                if ((Math.abs(error) >= WALL_DISTANCE_THRESHOLD) &&
                        ((Math.abs(angleOffset) < WALL_TRACKING_MAX_HEADING_OFFSET) || (error * angleOffset * distance < 0))) {

                    steer = getSteer(error, P_WALL_TRACKING_COEFF);

                    leftSpeed = speed - steer;
                    rightSpeed = speed + steer;

                    // Normalize speeds if any one exceeds +/- 1.0;
                    max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                    if (max > 1.0) {
                        leftSpeed /= max;
                        rightSpeed /= max;
                    }

                    robot.frontLeftMotor.setPower(leftSpeed);
                    robot.frontRightMotor.setPower(rightSpeed);
                    robot.backLeftMotor.setPower(leftSpeed);
                    robot.backRightMotor.setPower(rightSpeed);

                    if (ct3 == 0) {
                        fileLogger.logLine("error=" + error
                                + " leftspeed=" + String.format(Double.toString(leftSpeed), "%5.2f")
                                + " rightSpeed=" + String.format(Double.toString(rightSpeed), "%5.2f"));
                    }
                } else {
                    robot.frontLeftMotor.setPower(speed);
                    robot.frontRightMotor.setPower(speed);
                    robot.backRightMotor.setPower(speed);
                    robot.backLeftMotor.setPower(speed);
                }

                if (bBeaconDetection) {

                    // check color sensor. If it reaches beacon then stop robot.
                    red = robot.beaconColorSensor.blue();
                    green = robot.beaconColorSensor.green();
                    blue = robot.beaconColorSensor.red();

                    // any 0 reading might means out of range
                    if (red * green * blue == 0) {
                        continue;
                    }

//                    if (ct1 == 0)
//                    {
//                        fileLogger.logLine("red=" + red + " green=" + green + " blue=" + blue);
//                    }
//                    ct1++;
//                    if (ct1 > 500) {
//                        ct1 = 0;
//                    }

                    redCheck += ((red - green) * (red - blue) - redCheck) * COLOR_COEFF;
                    if (redCheck > redMax) {

                        redMax = redCheck;
                        passRedCounter = 0;
                    } else {
                        passRedCounter++;
                    }

                    blueCheck += ((blue - red) * (blue - green) - blueCheck) * COLOR_COEFF;
                    if (blueCheck > blueMax) {

                        blueMax = blueCheck;
                        passBlueCounter = 0;
                    } else {
                        passBlueCounter++;
                    }

                    if ((redMax > RED_TRESHOLD)
                            && (passRedCounter > 200)) {
                        // Red beacon reached
                        fileLogger.logLine("--- red light! redMax=" + redMax);

                        lightPassed++;
                        if (lightPassed > 1) {
                            blueMax = 0;

                            // Stop all motion;
                            robot.frontLeftMotor.setPower(0);
                            robot.frontRightMotor.setPower(0);
                            robot.backLeftMotor.setPower(0);
                            robot.backRightMotor.setPower(0);

//                        // We are blue team
//                        robot.pusherR.setPosition(PUSHER_R_OUT);
//
//                        //wait servo to finish
//                        sleep(500);
//
//                        // Retrieve the pusher
//                        robot.pusherR.setPosition(PUSHER_R_IN);
//
//                        //wait servo to finish
//                        sleep(500);

//                        fileLogger.logLine("pushed blue button on the right!");
                            break;
                        }
                    }

                    if ((blueMax > BLUE_TRESHOLD)
                            && (passBlueCounter > 200)) {
                        // Blue beacon reached
                        fileLogger.logLine("--- blue lght! blueMax=" + blueMax);

                        lightPassed++;
                        if (lightPassed > 1) {
                            redMax = 0;

                            // Stop all motion;
                            robot.frontLeftMotor.setPower(0);
                            robot.frontRightMotor.setPower(0);
                            robot.backLeftMotor.setPower(0);
                            robot.backRightMotor.setPower(0);

                            // We are the blue team
//                        robot.pusherL.setPosition(PUSHER_L_OUT);
//
//                        //wait servo to finish
//                        sleep(500);
//
//                        // Retrieve the pusher
//                        robot.pusherL.setPosition(PUSHER_L_IN);
//
//                        //wait servo to finish
//                        sleep(500);

//                        fileLogger.logLine("pushed blue button on the left!");

                            break;
                        }
                    }

                    if (ct1 == 0) {
                        fileLogger.logLine("redMax=" + redMax + " blueMax=" + blueMax);
                        fileLogger.logLine("RGB= " + red + " " + green + " " + blue);
                    }
                    ct1++;
                    if (ct1 > 500) {
                        ct1 = 0;
                    }

                    // Display drive status for the driver.
                    telemetry.addData("RGB=", "%d %d %d", red, green, blue);
                    telemetry.update();
                }
            }

            // Stop all motion;
            robot.frontLeftMotor.setPower(0);
            robot.frontRightMotor.setPower(0);
            robot.backLeftMotor.setPower(0);
            robot.backRightMotor.setPower(0);

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

            // check color again to see if robot is off position
//            red = robot.beaconColorSensor.red();
//            green = robot.beaconColorSensor.green();
//            blue = robot.beaconColorSensor.blue();
//
//            if (redMax == 0) {
//                fileLogger.logLine("RGB= " + red + " " + green + " " + blue);
//                fileLogger.logLine("(blue-red)*(blue-green)/blueMax=" + (blue - red) * (blue - green) / (double) blueMax);
//                // blue light detected
//                if ((blue - red) * (blue - green) / (double) blueMax < 0.85) {
//                    // overshoot the light
//                    return (true);
//                }
//            }
//
//            if (blueMax == 0) {
//                fileLogger.logLine("RGB= " + red + " " + green + " " + blue);
//                fileLogger.logLine("(red-blue)*(red-green)/redMax=" + (red - blue) * (red - green) / (double) redMax);
//                // red light detected
//                if ((red - blue) * (red - green) / (double) redMax < 0.85) {
//                    // overshoot the light
//                    return (true);
//                }
//            }
        }

        return (false);
    }

    public void ColorDetectionAndButtonPushing() {

        ElapsedTime holdTimer = new ElapsedTime();
        double holdTime = 2;  // 2 second timeout
        int red, green, blue;
        int redCheck = 0, blueCheck = 0;

        robot.beaconColorSensor.enableLed(false); //led OFF

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (holdTimer.time() < holdTime) {

            red = robot.beaconColorSensor.blue();
            green = robot.beaconColorSensor.green();
            blue = robot.beaconColorSensor.red();

            telemetry.addData("ColorRGB:: ", "%d %d %d", red, green, blue);
//            telemetry.update();
//           fileLogger.logLine("Alpha " + robot.beaconColorSensor.alpha());
//            fileLogger.logLine("Red " + robot.beaconColorSensor.red());
//            fileLogger.logLine("Blue " + robot.beaconColorSensor.blue());
//            fileLogger.logLine("Green " + robot.beaconColorSensor.green());

            redCheck = (red - blue) * (red - green);
            blueCheck = (blue - red) * (blue - green);

            telemetry.addData("ColorRC&BC :: ", "%d %d", redCheck, blueCheck);

            telemetry.update();

            // red color detected
            if (redCheck > RED_TRESHOLD) {

                // We are blue team
                robot.pusherR.setPosition(PUSHER_R_OUT);

                //wait servo to finish
                sleep(1000);

                // Retrieve the pusher
                robot.pusherR.setPosition(PUSHER_R_IN);

                //wait servo to finish
                sleep(500);

                fileLogger.logLine("--- red light detected and blue button pushed. redCheck=" + redCheck);
                break;
            }

            // blue color detected
            if (blueCheck > BLUE_TRESHOLD) {

                // We are the blue team
                robot.pusherL.setPosition(PUSHER_L_OUT);

                //wait servo to finish
                sleep(1000);

                // Retrieve the pusher
                robot.pusherL.setPosition(PUSHER_L_IN);

                //wait servo to finish
                sleep(500);

                fileLogger.logLine("--- blue light detected and blue button pushed. blueCheck=" + blueCheck);
                break;
            }

            sleep(10);
            idle();
        }
    }
}