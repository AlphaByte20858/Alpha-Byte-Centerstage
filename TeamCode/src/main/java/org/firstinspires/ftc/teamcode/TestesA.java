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
    Boolean IsOpen = false;
    ElapsedTime viao = new ElapsedTime();


    public void init() {
        SD = hardwareMap.get(Servo.class, "SD");
    }

    @Override
    public void loop() {

        if (gamepad2.left_bumper ){
                SD.setPosition(0.2);
            }
        else if(gamepad2.right_bumper) {

            SD.setPosition(0);
            IsOpen = true;
            viao.reset();
        }
        if (gamepad2.a){
            SD.setPosition(0.5);
        }


        if (IsOpen && viao.seconds() > 1){
            SD.setPosition(-0.3);
        }
    }
}