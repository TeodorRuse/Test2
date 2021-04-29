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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name="SovaAutonomous", group="Linear Opmode")
//@Disabled
public class SovaAutonomous extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx driveMotorR = null;
    private DcMotorEx driveMotorL = null;
    private DcMotorEx shooterMotor = null;
    private DcMotor conveyorMotor = null;
    private DcMotor intakeMotor = null;
    private DcMotor wobbleMotor = null;
    Servo wobbleServo;


    //INITIALISE GYROSCOPE------------------------------------------------------------------------------------
    BNO055IMU gyro ;
    Orientation angle;

    OpenCvCamera webcam;

    private boolean isIntakeOn = false;
    private boolean isWobbleOff = false;


    // DRIVE TRAIN INITIALISATION------------------------------------------------------------------------------
    static final int        MOTOR_TICK_COUNTS = 560; //28 baseTick * 20 gearReduction (4*5);
    static final double     WHEEL_DIAMETER_INCHES   = 3.54 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (MOTOR_TICK_COUNTS) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     DRIVE_SPEED             =  0.8; //0.8;
    static final double     TURN_SPEED              = 0.7; //0.9;


    static final double     HEADING_THRESHOLD       = 2.6 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            =  0.035;     // Larger is more responsive, but also less stable org 0.1  for TurnSpeed 0.6 is 0.05 a lottle smaller
    static final double     P_DRIVE_COEFF           =  0.035;     // Larger is more responsive, but also less stable org 0.15 for Drive Speed 0.06 is 0.05


    final int X_LEFT = 120;
    final int X_RIGHT = 150;
    final int Y_UP = 175;
    final int Y_MIDDLE = 165;
    final int Y_DOWN = 172;


    @Override
    public void runOpMode() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
        gyro.initialize(parameters);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);;
        webcam.openCameraDevice();

        // Loading pipeline
       RingPipeline visionPipeline = new RingPipeline();
        webcam.setPipeline(visionPipeline);

        // Start streaming the pipeline
        webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);

        //DRIVE TRAIN INITIALISE==========================================================
        driveMotorR = hardwareMap.get(DcMotorEx.class, "driveMotorR");
        driveMotorL = hardwareMap.get(DcMotorEx.class, "driveMotorL");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        conveyorMotor = hardwareMap.get(DcMotor.class, "conveyorMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        wobbleMotor = hardwareMap.get(DcMotor.class, "wobbleMotor");
        wobbleServo = hardwareMap.get(Servo.class, "wobbleServo");

        driveMotorR.setDirection(DcMotorEx.Direction.REVERSE);
        driveMotorL.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor.setDirection((DcMotorSimple.Direction.REVERSE));
        conveyorMotor.setDirection((DcMotorSimple.Direction.REVERSE));
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        wobbleMotor.setDirection(DcMotor.Direction.FORWARD);

        driveMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveMotorL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        driveMotorR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //DRIVE TRAIN INITIALISE==========================================================

        telemetry.addData("Status", "Initialized");
        telemetry.update();



        waitForStart();

        /*if(visionPipeline.ring4 == 0) {
            wobbleServo.setPosition(0.3);
        gyroTurn(TURN_SPEED, 349);
        gyroDrive(DRIVE_SPEED, 103, 349);
        gyroTurn(TURN_SPEED, 29);
        gyroDrive(DRIVE_SPEED, 44, 29);
        gyroTurn(TURN_SPEED, 0);
        gyroDrive(DRIVE_SPEED, 2, 0);
        shoot(4);
        gyroTurn(TURN_SPEED, 348);
        gyroDrive(DRIVE_SPEED, 120, 348);
        releaseWobble(true);
        gyroTurn(TURN_SPEED, 164);
        gyroDrive(DRIVE_SPEED, 120m, 164);

        }
        else if(visionPipeline.ring1 == 0) {
           wobbleServo.setPosition(0.3);
        gyroTurn(TURN_SPEED, 346);
        gyroDrive(DRIVE_SPEED, 104, 346);
        gyroTurn(TURN_SPEED, 38);
        gyroDrive(DRIVE_SPEED, 55, 38);
        gyroTurn(TURN_SPEED, 0);
        gyroDrive(DRIVE_SPEED, 6, 0);
        gyroTurn(TURN_SPEED, 0);
        gyroDrive(DRIVE_SPEED, -23, 0);
        shoot(4);
        gyroTurn(TURN_SPEED, 10);
        gyroDrive(DRIVE_SPEED, 74, 10);
        releaseWobble(true);
        gyroDrive(DRIVE_SPEED, -15, 10);
        }
        else
        {
            wobbleServo.setPosition(0.3);
            gyroTurn(TURN_SPEED, 2);
            gyroDrive(DRIVE_SPEED, 127, 2);
            shoot(4);
            gyroTurn(TURN_SPEED, 350);
            gyroDrive(DRIVE_SPEED, 28, 350);
            releaseWobble(true);
            gyroTurn(TURN_SPEED, 158);
            gyroDrive(DRIVE_SPEED, 102, 158);
            releaseWobble(false);
            gyroTurn(TURN_SPEED, 332);
            gyroDrive(DRIVE_SPEED, 101, 332);
            releaseWobble(true);

        /*wobbleServo.setPosition(0.3);
        gyroTurn(TURN_SPEED, 0);
        gyroDrive(DRIVE_SPEED, 144, 0);
        shoot(4);
        gyroTurn(TURN_SPEED, 325);
        gyroDrive(DRIVE_SPEED, 29, 325);
        releaseWobble(true);
        gyroTurn(TURN_SPEED, 154);
        gyroDrive(DRIVE_SPEED, 116, 154);
        releaseWobble(false);
        gyroTurn(TURN_SPEED, 339);
        gyroDrive(DRIVE_SPEED, 113, 339);
        releaseWobble(true);

    }*/


        while(opModeIsActive()) {
            telemetry.addData("Ring 1:", visionPipeline.ring1); // Will return 0 if there is 1 ring, otherwise 1
            telemetry.addData("Ring 4:", visionPipeline.ring4); // Will return 0 if there is 4 rings, otherwise 1
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.update();
        }
    }

    public void shoot(int rings)
    {
        if(rings == 1) {
            conveyorMotor.setPower(-0.45);
            shooterMotor.setVelocity(-1000);
            sleep(500);
            shooterMotor.setVelocity(1820);
            sleep(750);
            conveyorMotor.setPower(1);
            intakeMotor.setPower(1);
            sleep(1800);
            shooterMotor.setVelocity(0);
            conveyorMotor.setPower(0);
            intakeMotor.setPower(0);
            sleep(200);
        }
        else if(rings == 4)
        {
            conveyorMotor.setPower(-0.45);
            shooterMotor.setVelocity(-1000);
            sleep(500);
            shooterMotor.setVelocity(1820);
            sleep(750);
            conveyorMotor.setPower(1);
            intakeMotor.setPower(1);
            sleep(2500);
            shooterMotor.setVelocity(0);
            conveyorMotor.setPower(0);
            intakeMotor.setPower(0);
            sleep(200);
        }
    }

    public void intake(int rings)
    {
        if(rings == 1) {
            sleep(200);
            angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XZY, AngleUnit.DEGREES);
            gyroDrive(0.3, -30, -angle.secondAngle);
            intakeMotor.setPower(1);
            conveyorMotor.setPower(1);
            gyroDrive(0.3, -30, -angle.secondAngle);
            sleep(500);
            conveyorMotor.setPower(0);
            gyroDrive(0.5, 60, -angle.secondAngle);
            intakeMotor.setPower(0);
        }
        else if(rings==4)
        {
            sleep(200);
            angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XZY, AngleUnit.DEGREES);
            gyroDrive( 1,-30,  -angle.secondAngle);
            intakeMotor.setPower(1);
            conveyorMotor.setPower(1);
            gyroDrive(0.3,-30,  -angle.secondAngle);
            sleep(500);
            intakeMotor.setPower(-1);
            sleep(400);
            intakeMotor.setPower(1);
            //conveyorMotor.setPower(0);
            gyroDrive(0.5,60, -angle.secondAngle);
            intakeMotor.setPower(0);
            conveyorMotor.setPower(0);
        }
    }

    public void releaseWobble(boolean release)
    {
        if(!release)
        {
            sleep(100);
            wobbleServo.setPosition(-0.9);
            wobbleMotor.setPower(0.5);
            sleep(900);
            wobbleMotor.setPower(0);
            sleep(1200);
            wobbleServo.setPosition(0.3);
            sleep(200);
            wobbleMotor.setPower(-0.7);
            sleep(1500);
            wobbleMotor.setPower(0);
            wobbleServo.setPosition(0.2);
        }
        else
        {
            sleep(100);
            wobbleMotor.setPower(0.6);
            sleep(1300);
            wobbleServo.setPosition(-0.9);
            sleep(700);
            wobbleMotor.setPower(-0.5);
            sleep(1000);
            wobbleMotor.setPower(0);
        }
    }

    /*public void releaseWobble(boolean release)
    {
        if(!release)
        {
//
           /* wobbleServo.setPosition(0.3);
            sleep(200);
            wobbleMotor.setPower(-0.6);
            sleep(1500);
            wobbleMotor.setPower(0);
            wobbleServo.setPosition(0.2);

            wobbleMotor.setPower(-0.6);
            sleep(1500);
            wobbleMotor.setPower(0);
        }
        else
        {
           /* sleep(100);
            wobbleMotor.setPower(0.6);
            sleep(1500);
            wobbleServo.setPosition(0);
            wobbleMotor.setPower(-0.6);
            sleep(1500);
            wobbleMotor.setPower(0);

            wobbleMotor.setPower(0.6);
            sleep(1500);
            wobbleMotor.setPower(0);
        }
    }*/




    class RingPipeline extends OpenCvPipeline {


        // Working Mat variables
        Mat YCrCb = new Mat(); // This will store the whole YCrCb channel
        Mat Cb = new Mat(); // This will store the Cb Channel (part from YCrCb)
        Mat tholdMat = new Mat(); // This will store the threshold

        // Drawing variables
        Scalar GRAY = new Scalar(220, 220, 220); // RGB values for gray.
        Scalar GREEN = new Scalar(0, 255, 0); // RGB values for green.

        // Variables that will store the results of our pipeline
        public int ring1;
        public int ring4;

        // Space which we will annalise data
        public Point BigSquare1 = new Point(X_LEFT, Y_UP);
        public Point BigSquare2 = new Point(X_RIGHT, Y_DOWN);

        public Point SmallSquare1 = new Point(X_LEFT, Y_MIDDLE);
        public Point SmallSquare2 = new Point(X_RIGHT, Y_DOWN);

        @Override
        public Mat processFrame(Mat input) {

            // Img processing
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_BGR2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
            Imgproc.threshold(Cb, tholdMat, 150, 255, Imgproc.THRESH_BINARY_INV);

            // Drawing Points
            int BigSquarePointX = (int) ((BigSquare1.x + BigSquare2.x) / 2);
            int BigSquarePointY = (int) ((BigSquare1.y + SmallSquare1.y) / 2);

            int SmallSquarePointX = (int) ((SmallSquare1.x + SmallSquare2.x) / 2);
            int SmallSquarePointY = (int) ((SmallSquare1.y + SmallSquare2.y) / 2);

            // Point BigSquarePoint = new Point((int)((BigSquare1.x + BigSqare2.x) / 2),(int)((BigSquare1.y + SmallSquare1.y) / 2));
            // Point SmallSquarePoint = new Point((int)((SmallSquare1.x + SmallSquare2.x) / 2),(int)((SmallSquare1.y + SmallSquare2.y) / 2));

            double[] bigSquarePointValues = tholdMat.get(BigSquarePointY, BigSquarePointX);
            double[] smallSquarePointValues = tholdMat.get(SmallSquarePointY, SmallSquarePointX);

            ring4 = (int) bigSquarePointValues[0];
            ring1 = (int) smallSquarePointValues[0];

            // Big Square
            Imgproc.rectangle(
                    input,
                    BigSquare1,
                    BigSquare2,
                    GRAY,
                    1
            );

            // Small Square
            Imgproc.rectangle(
                    input,
                    SmallSquare1,
                    SmallSquare2,
                    GRAY,
                    1
            );

            // Big Square Point
            Imgproc.circle(
                    input,
                    new Point(BigSquarePointX, BigSquarePointY),
                    2,
                    GRAY,
                    1
            );

            // Small Square Point
            Imgproc.circle(
                    input,
                    new Point(SmallSquarePointX, SmallSquarePointY),
                    2,
                    GRAY,
                    1
            );

            // Change colors if the pipeline detected something

            if (ring1 == 0 && ring4 == 0) {
                Imgproc.rectangle(
                        input,
                        BigSquare1,
                        BigSquare2,
                        GREEN,
                        1
                );
                Imgproc.circle(
                        input,
                        new Point(BigSquarePointX, BigSquarePointY),
                        2,
                        GREEN,
                        1
                );
            }
            if (ring1 == 0) {
                Imgproc.rectangle(
                        input,
                        SmallSquare1,
                        SmallSquare2,
                        GREEN,
                        1
                );
                Imgproc.circle(
                        input,
                        new Point(SmallSquarePointX, SmallSquarePointY),
                        2,
                        GREEN,
                        1
                );
            }

            return input;
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

        angle = 360-angle;
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
                /*
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      driveMotorL.getCurrentPosition(),
                        driveMotorR.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
                */
            }

            // Stop all motion;
            driveMotorL.setPower(0);
            driveMotorR.setPower(0);

            // Turn off RUN_TO_POSITION
            driveMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        sleep(600);
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

        angle = 360 - angle;
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
        sleep(600);
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
        robotError = targetAngle + angle.secondAngle;
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