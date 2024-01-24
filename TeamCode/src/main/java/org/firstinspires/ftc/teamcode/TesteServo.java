package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="TesteServo", group="LinearOpMoDE")
public class TesteServo extends LinearOpMode {
    Servo servoArm;
    public void runOpMode(){

        servoArm = hardwareMap.get(Servo.class, "servoArm");
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("servor Valor", servoArm.getPosition());
            telemetry.update();
        }
    }
}

