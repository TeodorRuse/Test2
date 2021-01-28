package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@TeleOp(name="Simple driving", group="Tele")
public class SimpleDriving extends OpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;

    double leftMotorPower;
    double rightMotorPower;

    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("drivetrain_left_motor");
        rightMotor = hardwareMap.dcMotor.get("drivetrain_right_motor");

        rightMotor.setDirection(REVERSE);
    }

    @Override
    public void loop() {
        leftMotorPower = gamepad1.left_stick_y;
        rightMotorPower = gamepad1.right_stick_y;
        leftMotor.setPower(leftMotorPower);
        rightMotor.setPower(rightMotorPower);
    }
}