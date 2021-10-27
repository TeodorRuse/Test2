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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="AutonomieExmplu", group="Linear Opmode")
//@Disabled
public class AutonomieExemplu extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx driveMotorR = null;
    private DcMotorEx driveMotorL = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        driveMotorL  = hardwareMap.get(DcMotorEx.class, "left_drive");
        driveMotorR = hardwareMap.get(DcMotorEx.class, "right_drive");

        driveMotorL.setDirection(DcMotor.Direction.FORWARD);
        driveMotorR.setDirection(DcMotor.Direction.REVERSE);

       /* driveMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            drive(1,1)
            sleep(450);
            drive(0,0);
            drive(0,0.3);
            sleep(140);
            drive(0,0);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }

    public void drive(double driveRight, double driveLeft)
    {
        driveMotorL.setPower(driveRight);
        driveMotorR.setPower(driveLeft);
    }
}
