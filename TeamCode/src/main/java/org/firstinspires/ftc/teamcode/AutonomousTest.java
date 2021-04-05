/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="AutonomousTest", group="Linear Opmode")
//@Disabled
public class AutonomousTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx driveMotorR = null;
    private DcMotorEx driveMotorL = null;


    //INITIALISE GYROSCOPE------------------------------------------------------------------------------------
    BNO055IMU gyro ;
    Orientation angle;


    // DRIVE TRAIN INITIALISATION------------------------------------------------------------------------------
    static final int        MOTOR_TICK_COUNTS = 560; //28 baseTick * 20 gearReduction (4*5);
    static final double     WHEEL_DIAMETER_INCHES   = 3.54 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (MOTOR_TICK_COUNTS) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     DRIVE_SPEED             = 0.4;
    static final double     TURN_SPEED              = 0.35;


    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyroo
    static final double     P_TURN_COEFF            = 0.05;     // Larger is more responsive, but also less stable org 0.1
    static final double     P_DRIVE_COEFF           = 0.1;     // Larger is more responsive, but also less stable org 0.15



    @Override
    public void runOpMode() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
        gyro.initialize(parameters);

        if (gyro.isGyroCalibrated()){
            telemetry.addData("Gyro has Calibrated", 0);
            telemetry.update();
        }


        //DRIVE TRAIN INITIALISE==========================================================
        driveMotorR = hardwareMap.get(DcMotorEx.class, "driveMotorR");
        driveMotorL = hardwareMap.get(DcMotorEx.class, "driveMotorL");

        driveMotorR.setDirection(DcMotorEx.Direction.FORWARD);
        driveMotorL.setDirection(DcMotorSimple.Direction.REVERSE);

        driveMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveMotorL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        driveMotorR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //DRIVE TRAIN INITIALISE==========================================================

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        gyroTurn(TURN_SPEED, 0);

        waitForStart();

        //angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XZY, AngleUnit.DEGREES);

        /*encoderDrive(DRIVE_SPEED, 50);
        encoderDrive(DRIVE_SPEED, -30);*/

        /*gyroDrive(DRIVE_SPEED, 70, 0);
        gyroTurn(TURN_SPEED, 90);
        gyroDrive(DRIVE_SPEED, 50, 45);*/

        //just try soinning in a circle to see if shit works, still dont know how angle workse

        gyroDrive(DRIVE_SPEED, 170, 0);
        gyroTurn(TURN_SPEED, 90);
        gyroDrive(DRIVE_SPEED, 110,0);
        gyroTurn(DRIVE_SPEED, 180);
        gyroDrive(DRIVE_SPEED, 170, 0);
        gyroTurn(TURN_SPEED, 270);
        gyroDrive(DRIVE_SPEED, 110,0);
        gyroTurn(DRIVE_SPEED, 360);



        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

    }


    public void encoderDrive(double speed, double distance){
            int newLeftTarget;
            int newRightTarget;

            distance /= 2.54; // convert cm to inch

            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newLeftTarget = driveMotorL.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
                newRightTarget = driveMotorR.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
                driveMotorL.setTargetPosition(newLeftTarget);
                driveMotorR.setTargetPosition(newRightTarget);

                // Turn On RUN_TO_POSITION
                driveMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                driveMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                driveMotorL.setPower(Math.abs(speed));
                driveMotorR.setPower(Math.abs(speed));

                while (opModeIsActive() && (driveMotorL.isBusy() && driveMotorR.isBusy())) {
                    //do nothing but can print
                }

                // Stop all motion;
                driveMotorL.setPower(0);
                driveMotorR.setPower(0);

                // Turn off RUN_TO_POSITION
                driveMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                driveMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
    }



    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (originally in inches, now in cm) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        distance /= 2.54; // convert cm to inch

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = driveMotorL.getCurrentPosition() + moveCounts;
            newRightTarget = driveMotorR.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            driveMotorL.setTargetPosition(newLeftTarget);
            driveMotorR.setTargetPosition(newRightTarget);

            driveMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            driveMotorL.setPower(speed);
            driveMotorR.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (driveMotorL.isBusy() && driveMotorL.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                driveMotorL.setPower(leftSpeed);
                driveMotorR.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      driveMotorL.getCurrentPosition(),
                        driveMotorR.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            driveMotorL.setPower(0);
            driveMotorR.setPower(0);

            // Turn off RUN_TO_POSITION
            driveMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        driveMotorL.setPower(leftSpeed);
        driveMotorR.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        sleep(50);
        angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XZY, AngleUnit.DEGREES);
        // calculate error in -179 to +180 range  (
        robotError = targetAngle - angle.secondAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
}
//merge