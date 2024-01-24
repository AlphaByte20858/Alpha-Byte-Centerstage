package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.tensorflow.lite.task.vision.core.BaseVisionTaskApi;

@Autonomous (name = "Thiago", group = "LinearOpMode")
public class testeTH extends LinearOpMode {

    DcMotorEx M0, M1, M2, M3, Linear, Intake;
    ServoImplEx garra, SD;

    public void runOpMode() {

        M0 = hardwareMap.get(DcMotorEx.class, "MEF");
        M1 = hardwareMap.get(DcMotorEx.class, "MDF");
        M2 = hardwareMap.get(DcMotorEx.class, "MET");
        M3 = hardwareMap.get(DcMotorEx.class, "MDT");
        Linear = hardwareMap.get(DcMotorEx.class, "Linear");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        garra = hardwareMap.get(ServoImplEx.class, "garra");
        SD = hardwareMap.get(ServoImplEx.class, "SD");

        M0.setDirection(DcMotorEx.Direction.REVERSE);
        M1.setDirection(DcMotorEx.Direction.REVERSE);
        M2.setDirection(DcMotorEx.Direction.REVERSE);
        M3.setDirection(DcMotorEx.Direction.REVERSE);
        Linear.setDirection(DcMotorSimple.Direction.REVERSE);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        garra.setDirection(ServoImplEx.Direction.REVERSE);
        SD.setDirection(ServoImplEx.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){


        }

    }

}

