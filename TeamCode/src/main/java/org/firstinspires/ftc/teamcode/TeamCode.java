package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@TeleOp(name="Simple driving", group="Tele")
public class TeamCode extends OpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;
    double power = 0.5;

    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("drivetrain_left_motor");
        rightMotor = hardwareMap.dcMotor.get("drivetrain_right_motor");

        rightMotor.setDirection(REVERSE);
    }

    @Override
    public void loop() {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }
}