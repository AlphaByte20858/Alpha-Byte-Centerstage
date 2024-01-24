//usar para testes de in take+linear (sistema de pontuação no backdrop)
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp (name= "TestesA", group = "OpMode")
public class TestesA extends OpMode {
    DcMotorEx ITK, Linear;
    Servo garra;
    public void init() {
        ITK = hardwareMap.get(DcMotorEx.class, "Intake");
        Linear = hardwareMap.get(DcMotorEx.class, "Linear");
        garra = hardwareMap.get(Servo.class, "garra");

        ITK.setDirection(DcMotorEx.Direction.REVERSE);
        Linear.setDirection(DcMotorEx.Direction.REVERSE);
        garra.setDirection(Servo.Direction.REVERSE);

    }

    @Override
    public void loop() {
        Flinear();
        INTAKE();
        Garra();
    }

    public void INTAKE(){
        ITK.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
    }

    public void Flinear() {
        if (gamepad1.right_bumper)  {
            Linear.setPower(1);
        }
        else if (gamepad1.left_bumper) {
            Linear.setPower(-1);
        }
        else {
            Linear.setPower(0);
        }
    }

    public void Garra(){
        if (gamepad1.y){
            garra.setPosition(0.19);
        }
        else if (gamepad1.x){
            garra.setPosition(0.84);
        }
    }
}