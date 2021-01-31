package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@TeleOp(name="Telecomanda", group="Tele")
public class Telecomanda extends OpMode {
    //motoarele drivetrainului
    DcMotor leftMotor;
    DcMotor rightMotor;
    double leftMotorPower;
    double rightMotorPower;

    DcMotor intake; //HD Hex Motor de la intake
    boolean intakeActivate;

    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("drivetrain_left_motor");
        rightMotor = hardwareMap.dcMotor.get("drivetrain_right_motor");

        intake = hardwareMap.dcMotor.get("intake_motor");

        rightMotor.setDirection(REVERSE);
    }

    @Override
    public void loop() {
        intakeActivate = gamepad1.b;
        leftMotorPower = -gamepad1.left_stick_y;
        rightMotorPower = -gamepad1.right_stick_y;

        leftMotor.setPower(leftMotorPower);
        rightMotor.setPower(rightMotorPower);

        if(intakeActivate = true) {
            intake.setPower(0.5);
        }
    }
}

//no