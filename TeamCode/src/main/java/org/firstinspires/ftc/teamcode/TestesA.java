//usar para testes de in take+linear (sistema de pontuação no backdrop)
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name= "TestesA", group = "OpMode")
public class TestesA extends OpMode {
    Servo SD;
    ServoControllerEx teste;
    public void init() {
        SD = hardwareMap.get(Servo.class, "SD");
        teste = hardwareMap.get(ServoControllerEx.class, "SD2");
    }

    @Override
    public void loop() {
        ElapsedTime viao = new ElapsedTime();
        if (gamepad2.left_bumper){
            SD.setPosition(0.2);
            viao.startTime();
        }
        if (viao.seconds() > 5){
                SD.setPosition(0);
        }
    }
}