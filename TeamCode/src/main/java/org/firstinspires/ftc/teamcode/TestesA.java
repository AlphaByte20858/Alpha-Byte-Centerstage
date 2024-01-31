//usar para testes de in take+linear (sistema de pontuação no backdrop)
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name= "TestesA", group = "OpMode")
public class TestesA extends OpMode {
    Servo sClaw;

    public void init() {
        sClaw = hardwareMap.get(Servo.class, "sClaw");
        sClaw.setDirection(Servo.Direction.REVERSE);
        sClaw.setPosition(0);
    }

    @Override
    public void loop(){
        if (gamepad1.a){
            sClaw.setPosition(0.25);
        }
        else if (gamepad1.b){
            sClaw.setPosition(0.5);
        }
        else if (gamepad1.x){
            sClaw.setPosition(0.75);
        }
        else if (gamepad1.y){
            sClaw.setPosition(1);
        }
        else if (gamepad1.right_bumper){
            sClaw.setDirection(Servo.Direction.FORWARD);
        }
        else if (gamepad1.left_bumper){
            sClaw.setDirection(Servo.Direction.REVERSE);
        }
    }
}