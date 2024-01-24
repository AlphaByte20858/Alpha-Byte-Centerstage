//esta programação ainda não foi testada até o momento e está sob desenvolvimento.
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name = "testes3", group = "OpMode")
public class testes3 extends LinearOpMode {

    DcMotor Motor0, Motor1, Motor2, Motor3 = null;
    ElapsedTime runtime = new ElapsedTime();

    public void runOpMode(){

        telemetry.addData("status", "initialize");
        telemetry.update();

        Motor0 = hardwareMap.get(DcMotor.class, "MEF");
        Motor1 = hardwareMap.get(DcMotor.class, "MDF");
        Motor2 = hardwareMap.get(DcMotor.class, "MET");
        Motor3 = hardwareMap.get(DcMotor.class, "MDT");

        Motor0.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor3.setDirection(DcMotorSimple.Direction.FORWARD);

        double valor_encoder0 = 0;
        double valor_encoder1 = 0;
        double valor_encoder2 = 0;
        double valor_encoder3 = 0;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()){
            Motor0.setPower(gamepad1.left_stick_y);
            valor_encoder0 = Motor0.getCurrentPosition();

            Motor1.setPower(gamepad1.left_stick_y);
            valor_encoder1 = Motor0.getCurrentPosition();

            Motor2.setPower(gamepad1.left_stick_y);
            valor_encoder2 = Motor0.getCurrentPosition();

            Motor3.setPower(gamepad1.left_stick_y);
            valor_encoder3 = Motor0.getCurrentPosition();

        }

    }

}