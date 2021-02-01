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

    DcMotor intakeConveyer; //HD Hex Motor de la intake si conveyer
    boolean intakeConveyerActivate;

    DcMotor shooter;
    double shooterPower;

    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("drivetrain_left_motor");
        rightMotor = hardwareMap.dcMotor.get("drivetrain_right_motor");

        intakeConveyer = hardwareMap.dcMotor.get("intake_motor");

        shooter = hardwareMap.dcMotor.get("shooter");

        rightMotor.setDirection(REVERSE);
    }

    //Desi intakeul si conveyerul sunt actionate de acelasi motor, diferenta de putere si viteaza este data de sprocketuri

    @Override
    public void loop() {
        intakeConveyerActivate = gamepad1.b; // cand se apasa pe "b" pe gamepad se activeaza intakeul + conveyer
        leftMotorPower = -gamepad1.left_stick_y;
        rightMotorPower = -gamepad1.right_stick_y;

        leftMotor.setPower(leftMotorPower); //am setat puterea motoarelor in functie de cat se misca stickurile
        rightMotor.setPower(rightMotorPower);

        shooterPower = gamepad1.right_trigger; //am setat puterea shooterului pe trigger
        shooter.setPower(shooterPower);

        if(intakeConveyerActivate = true) { //se testeaza daca tasta "b" este apasata pe gamepad
            intakeConveyer.setPower(0.5);
        }
    }
}
