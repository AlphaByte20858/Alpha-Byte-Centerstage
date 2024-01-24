//esta programação ainda não foi testada até o momento e está sob desenvolvimento.
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name = "testes3", group = "LinearOpMode")
public class testes3 extends LinearOpMode {

    DcMotorEx Motor0, Motor1, Motor2, Motor3 = null;
    ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440; // CPR do motor, entre no site da fabricante
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;  // Redução entre motor e roda
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;  // Diâmetro da roda
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415); // Fator de conversão
    static final double     DRIVE_SPEED             = 0.6; // Com que tensão os motores se moverão para frente
    static final double     TURN_SPEED              = 0.5; // Com que tensão os motores farão o robô girar

    public void runOpMode(){

        telemetry.addData("status", "initialize");
        telemetry.update();

        Motor0 = hardwareMap.get(DcMotorEx.class, "MEF");
        Motor1 = hardwareMap.get(DcMotorEx.class, "MDF");
        Motor2 = hardwareMap.get(DcMotorEx.class, "MET");
        Motor3 = hardwareMap.get(DcMotorEx.class, "MDT");


        Motor0.setDirection(DcMotorEx.Direction.REVERSE);
        Motor1.setDirection(DcMotorEx.Direction.FORWARD);
        Motor2.setDirection(DcMotorEx.Direction.REVERSE);
        Motor3.setDirection(DcMotorEx.Direction.FORWARD);

        int motor0 = Motor0.getCurrentPosition()/1440;
        int motor1 = Motor1.getCurrentPosition()/1440;
        int motor2 = Motor2.getCurrentPosition()/1440;
        int motor3 = Motor3.getCurrentPosition()/1440;

        double valor_encoder0 = 0;
        double valor_encoder1 = 0;
        double valor_encoder2 = 0;
        double valor_encoder3 = 0;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()){
            Motor0.setTargetPosition(motor0 = 30);
            Motor1.setTargetPosition(motor1 = 30);
            Motor2.setTargetPosition(motor2 = 30);
            Motor3.setTargetPosition(motor3 = 30);

        }
    }
}