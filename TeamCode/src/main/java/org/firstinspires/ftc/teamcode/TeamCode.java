package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Simple driving", group="Tele")

public class TeamCode extends OpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;
    double power = 0.5;

    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("drivetrain_left_motor");
        rightMotor = hardwareMap.dcMotor.get("drivetrain_right_motor");
    }

    @Override
    public void loop() {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }
}