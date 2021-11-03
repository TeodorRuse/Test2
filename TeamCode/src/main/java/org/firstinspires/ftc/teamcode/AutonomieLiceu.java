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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


@TeleOp(name="AutoTest", group="Linear Opmode")
//@Disabled
public class AutonomousTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx driveMotorL = null;
    private DcMotorEx driveMotorR = null;

    static final int MOTOR_TICK_COUNTS = 560; //28 baseTick * 20 gearReduction (4*5);
    static final double WHEEL_DIAMETER_INCHES = 3.54;     // For figuring circumference
    static final double COUNTS_PER_INCH = (MOTOR_TICK_COUNTS) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double DRIVE_SPEED = 0.8; //0.8;
    static final double TURN_SPEED = 0.7; //0.9;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        driveMotorL = hardwareMap.get(DcMotorEx.class, "left_drive");
        driveMotorR = hardwareMap.get(DcMotorEx.class, "right_drive");

        driveMotorL.setDirection(DcMotor.Direction.REVERSE);
        driveMotorR.setDirection(DcMotor.Direction.FORWARD);

        driveMotorL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        driveMotorR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        if (opModeIsActive()) {
//            for(int i=0;i<4;i++) {
//                Drive(DRIVE_SPEED, 50, 0, 0.1);
//                Drive(DRIVE_SPEED, 0, 45, 0.1);
//                Drive(DRIVE_SPEED, 50, 0, 0.1);
//                Drive(DRIVE_SPEED, 0, 45, 0.1);
//                Drive(DRIVE_SPEED, 50, 0, 0.1);
//                Drive(DRIVE_SPEED, 0, 45, 0.1);
//                Drive(DRIVE_SPEED, 50, 0, 0.1);
//                Drive(DRIVE_SPEED, 0, 45, 0.1);
//            }
            //Drive(DRIVE_SPEED, 50, 0, 0.1);

            //Drive(DRIVE_SPEED, 0, 180, 0.1);

            //Drive(DRIVE_SPEED, 0, 360, 0.1);

            //Drive(DRIVE_SPEED, 0, 180, 0.1);

        }
    }

    public void Drive(double speed,
                      double distance, double angle, double radius) {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts1, moveCounts2;
        double distance1, distance2;
        double max;
        double leftSpeed;
        double rightSpeed;

        distance /= 2.54; // convert cm to inch
        radius /= 2.54;
        angle/=2;

        distance1 = (angle * 0.01745329) * radius + distance;
        distance2 = (angle * 0.01745329) * (radius + 7.87) + distance;

        telemetry.addData(" ", distance1);
        telemetry.addData(" ", distance2);
        telemetry.update();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts1 = (int) (distance1 * COUNTS_PER_INCH);
            moveCounts2 = (int) (distance2 * COUNTS_PER_INCH);
            newLeftTarget = driveMotorL.getCurrentPosition() + moveCounts2;
            newRightTarget =driveMotorR.getCurrentPosition() + moveCounts1;

            // Set Target and Turn On RUN_TO_POSITION
            driveMotorL.setTargetPosition(newLeftTarget);
            driveMotorR.setTargetPosition(newRightTarget);

            driveMotorL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            driveMotorR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            driveMotorL.setPower(speed);
            driveMotorR.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && (driveMotorL.isBusy() && driveMotorL.isBusy())) {

                leftSpeed = speed;
                rightSpeed = speed;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                driveMotorL.setPower(leftSpeed);
                driveMotorR.setPower(rightSpeed);
            }

            // Stop all motion;
            driveMotorL.setPower(0);
            driveMotorR.setPower(0);

            // Turn off RUN_TO_POSITION
            driveMotorL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            driveMotorR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            //driveMotorL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            //driveMotorR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        }
        //sleep(600);
    }


    /*int speedboy = gamepad1.left_stick_y;
    int speedboy = gamepad1.right_stick_y;
    int E = 20;
    public void drive(int R, int speedboy){
        driveMotorL.setPower((R)/(R-E)*speedboy);
        driveMotorL.setPower((R+E)/(R)*speedboy);

    }*/
//    public void Drive2(double speed,
//                      double distance1, double distance2) {
//
//        int newLeftTarget;
//        int newRightTarget;
//        int moveCounts1, moveCounts2;
//        double max;
//        double leftSpeed;
//        double rightSpeed;
//
//        distance1 /= 2.54; // convert cm to inch
//        distance2 /=2.54;
//
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//            moveCounts1 = (int) (distance1 * COUNTS_PER_INCH);
//            moveCounts2 = (int) (distance2 * COUNTS_PER_INCH);
//            newLeftTarget = driveMotorL.getCurrentPosition() + moveCounts2;
//            newRightTarget = driveMotorR.getCurrentPosition() + moveCounts1;
//
//            // Set Target and Turn On RUN_TO_POSITION
//            driveMotorL.setTargetPosition(newLeftTarget);
//            driveMotorR.setTargetPosition(newRightTarget);
//
//            driveMotorL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            driveMotorR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//
//            // start motion.
//            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
//            driveMotorL.setPower(speed);
//            driveMotorR.setPower(speed);
//
//            // keep looping while we are still active, and BOTH motors are running.
//            while (opModeIsActive() &&
//                    (driveMotorL.isBusy() && driveMotorL.isBusy())) {
//
//                leftSpeed = speed;
//                rightSpeed = speed;
//
//                // Normalize speeds if either one exceeds +/- 1.0;
//                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
//                if (max > 1.0) {
//                    leftSpeed /= max;
//                    rightSpeed /= max;
//                }
//
//                driveMotorL.setPower(leftSpeed);
//                driveMotorR.setPower(rightSpeed);
//            }
//
//            // Stop all motion;
//            driveMotorL.setPower(0);
//            driveMotorR.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            driveMotorL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//            driveMotorR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//
//        }
//        sleep(600);
//    }


}



