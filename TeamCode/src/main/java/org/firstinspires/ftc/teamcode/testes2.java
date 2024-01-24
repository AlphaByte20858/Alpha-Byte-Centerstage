package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "testes2", group = "OpMode")

public class testes2 extends OpMode {

    DcMotor CL;
    Servo GCL;

    public void init() {
        CL = hardwareMap.get(DcMotor.class, "CLIMB");
        GCL = hardwareMap.get(Servo.class, "GarraCLIMB");

        CL.setDirection(DcMotor.Direction.FORWARD);
        GCL.setDirection(Servo.Direction.REVERSE);
    }

    public void loop() {
        Climb();
        GarraC();

    }

    public void Climb() {
        if(gamepad1.a){
            CL.setPower(1);
        }
        else if (gamepad1.b){
            CL.setPower(gamepad1.right_trigger * 50);
        }
        else {
            CL.setPower(0);
        }
    }
    public void GarraC() {
        if (gamepad1.x) {
            GCL.setPosition(1);
        }
        else if (gamepad1.y) {
            GCL.setPosition(0);
        }
    }
}

