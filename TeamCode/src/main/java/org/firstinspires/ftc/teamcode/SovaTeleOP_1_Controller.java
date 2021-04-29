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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="SovaTeleOP(1_Controller)", group="Linear Opmode")
@Disabled
public class SovaTeleOP_1_Controller extends LinearOpMode {

    /*
        right stick = forward;
        left stick = right
        cross = intake; (toggle)
        left bumper = conveyour
        right bumper = conveyour + shooter;
        right arrow = shooter motor;
      need to add all buttons;
     */

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor driveMotorR = null;
    private DcMotor driveMotorL = null;
    private DcMotor intakeMotor = null;
    private DcMotor conveyorMotor = null;
    private DcMotorEx shooterMotor = null;
    private DcMotor wobbleMotor = null;
    Servo wobbleServo;

    private boolean isIntakeOn = false;
    private double dirveMotorSpeedBasic = 0.85;
    private boolean isWobbleOff = false;
    double powerDriveR;
    double powerDriveL;
    double intakePower;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Declare Motors----------------------------------------------------------------------------
        driveMotorR = hardwareMap.get(DcMotor.class, "driveMotorR");
        driveMotorL = hardwareMap.get(DcMotor.class, "driveMotorL");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        conveyorMotor = hardwareMap.get(DcMotor.class, "conveyorMotor");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        wobbleMotor = hardwareMap.get(DcMotor.class, "wobbleMotor");
        wobbleServo = hardwareMap.get(Servo.class, "wobbleServo");

        //Declare Directions and encoders-----------------------------------------------------------
        driveMotorR.setDirection(DcMotor.Direction.FORWARD);
        driveMotorL.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        conveyorMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        wobbleMotor.setDirection(DcMotor.Direction.FORWARD);

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Wait for Start----------------------------------------------------------------------------
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            //Drive Train---------------------------------------------------------------------------
            double drive = gamepad1.left_stick_y;
            double turn  =  -gamepad1.right_stick_x;
            powerDriveR    = Range.clip(drive + turn, -dirveMotorSpeedBasic, dirveMotorSpeedBasic) ;
            powerDriveL   = Range.clip(drive - turn, -dirveMotorSpeedBasic, dirveMotorSpeedBasic) ;

            driveMotorR.setPower(powerDriveR);
            driveMotorL.setPower(powerDriveL);

            //Wobble Servo--------------------------------------------------------------------------
            if(gamepad1.triangle){
                if(isWobbleOff)
                {
                    wobbleServo.setPosition(0.3);
                    isWobbleOff = false;
                    sleep(500);
                }
                else
                {
                    wobbleServo.setPosition(0.0);
                    isWobbleOff = true;
                    sleep(500);
                }
            }

            //Wobble Motor--------------------------------------------------------------------------
            if(gamepad1.dpad_up)
                wobbleMotor.setPower(0.5);
            else if(gamepad1.dpad_down)
                wobbleMotor.setPower(-0.5);
            else
                wobbleMotor.setPower(0);

            //Shooter motor------------------------------------------------------------------------
            if(gamepad1.right_bumper)
                shooterMotor.setVelocity(1820);
            else
                shooterMotor.setPower(0);

            //Conveyor Motor------------------------------------------------------------------------
            if(gamepad1.left_bumper)
                conveyorMotor.setPower(1.0);
            else if(gamepad2.square)
                conveyorMotor.setPower(-1);
            else
                conveyorMotor.setPower(0);

            //Intake Motor--------------------------------------------------------------------------
            if(gamepad1.cross) {
                isIntakeOn = !isIntakeOn;
                sleep(500);
            }

            if(isIntakeOn)
                intakePower = 0.9;
            else
                intakePower = 0;

            intakeMotor.setPower(intakePower);

            if(gamepad1.circle)
                intakeMotor.setPower(-1);
            else
                intakeMotor.setPower(intakePower);
            //END Intake Motor----------------------------------------------------------------------


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("ServoPosition = ", wobbleServo.getPosition());

            telemetry.update();
        }
    }
}
