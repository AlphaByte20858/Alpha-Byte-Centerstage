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
    DcMotorEx MDF, MEF, MDT, MET;

    public void init() {
        MDF = hardwareMap.get(DcMotorEx.class, "MDF");
        MEF = hardwareMap.get(DcMotorEx.class, "MEF");
        MDT = hardwareMap.get(DcMotorEx.class, "MDT");
        MET = hardwareMap.get(DcMotorEx.class, "MET");
        
        MET.setDirection(DcMotorSimple.Direction.REVERSE);
        MDT.setDirection(DcMotorSimple.Direction.FORWARD);
        MEF.setDirection(DcMotorSimple.Direction.REVERSE);
        MDF.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void loop() {
        if (gamepad2.a){
            MEF.setPower(1);
        }
        else if (gamepad2.b){
            MDF.setPower(1);
        }
        else if (gamepad2.x){
            MET.setPower(1);
        }
        else if (gamepad2.y){
            MDT.setPower(1);
        }
        else {
            MDF.setPower(0);
            MEF.setPower(0);
            MDT.setPower(0);
            MET.setPower(0);
        }
    }
}