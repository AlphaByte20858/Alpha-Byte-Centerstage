package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name = "Automonos teste", group = "LinearOpMode")
public class teste5 extends LinearOpMode {

    DcMotorEx M0, M1, M2, M3, Linear, Intake = null;
    ServoImplEx garra, SD = null;

    static final double COUNS_PER_MOTOR_REV = 560;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHELL_DIAMETER_INCHES = 1.96;
    double FatorDeConversao = (COUNS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHELL_DIAMETER_INCHES * Math.PI);

    public void runOpMode() {

        telemetry.addData("stats", "Initialize");
        telemetry.update();

        M0 = hardwareMap.get(DcMotorEx.class, "MEF");
        M1 = hardwareMap.get(DcMotorEx.class, "MDF");
        M2 = hardwareMap.get(DcMotorEx.class, "MET");
        M3 = hardwareMap.get(DcMotorEx.class, "MDT");
        Linear = hardwareMap.get(DcMotorEx.class, "Linear");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        garra = hardwareMap.get(ServoImplEx.class, "garra");
        SD = hardwareMap.get(ServoImplEx.class, "SD");

        M0.setDirection(DcMotorEx.Direction.REVERSE);
        M1.setDirection(DcMotorEx.Direction.FORWARD);
        M2.setDirection(DcMotorEx.Direction.REVERSE);
        M3.setDirection(DcMotorEx.Direction.FORWARD);
        Linear.setDirection(DcMotorEx.Direction.REVERSE);
        Intake.setDirection(DcMotorEx.Direction.REVERSE);
        garra.setDirection(ServoImplEx.Direction.REVERSE);
        SD.setDirection(ServoImplEx.Direction.REVERSE);

        M0.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        M1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        M2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        M3.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Linear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        addSetpoint(10);
        sleep(1000);
        addSetpoint(-10);
    }

    public void addSetpoint(int SetPoint) {

        int SetPoint0 = M0.getCurrentPosition() + (int) (SetPoint * FatorDeConversao);
        M0.setTargetPosition(SetPoint0);
        M0.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        M0.setPower(1);

        int SetPoint1 = M1.getCurrentPosition() + (int) (SetPoint * FatorDeConversao);
        M1.setTargetPosition(SetPoint1);
        M1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        M1.setPower(1);

        int SetPoint2 = M2.getCurrentPosition() + (int) (SetPoint * FatorDeConversao);
        M2.setTargetPosition(SetPoint2);
        M2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        M2.setPower(1);

        int SetPoint3 = M3.getCurrentPosition() + (int) (SetPoint * FatorDeConversao);
        M3.setTargetPosition(SetPoint3);
        M3.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        M3.setPower(1);

        while (M0.isBusy()) {
            telemetry.addData("Distancia percorrida", M0.getCurrentPosition() / FatorDeConversao);
            telemetry.update();
        }
        while (M1.isBusy()) {
            telemetry.addData("Distancia percorrida", M1.getCurrentPosition() / FatorDeConversao);
            telemetry.update();
        }
        while (M2.isBusy()) {
            telemetry.addData("Distancia percorrida", M2.getCurrentPosition() / FatorDeConversao);
            telemetry.update();
        }
        while (M3.isBusy()) {
            telemetry.addData("Distancia percorrida", M3.getCurrentPosition() / FatorDeConversao);
            telemetry.update();
        }
        M0.setPower(0);
        M0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(500);

        M1.setPower(0);
        M1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(500);

        M2.setPower(0);
        M2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(500);

        M3.setPower(0);
        M3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(500);
    }
}