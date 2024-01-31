package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name = "AutomonosAmistoso", group = "LinearOpMode")
public class AutonomusAmistoso extends LinearOpMode {

    DcMotorEx MEF, MDF, MET, MDT, MLS, MIT = null;
    ServoImplEx sClaw, sDrone = null;
    static final double COUNS_PER_MOTOR_REV = 560;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHELL_DIAMETER_INCHES = 1.96;
    double FatorDeConversao = (COUNS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHELL_DIAMETER_INCHES * Math.PI);

    public void runOpMode() {

        telemetry.addData("stats", "Initialize");
        telemetry.update();

        MEF = hardwareMap.get(DcMotorEx.class, "MEF");
        MDF= hardwareMap.get(DcMotorEx.class, "MDF");
        MET = hardwareMap.get(DcMotorEx.class, "MET");
        MDT = hardwareMap.get(DcMotorEx.class, "MDT");
        MLS = hardwareMap.get(DcMotorEx.class, "MLS");
        MIT = hardwareMap.get(DcMotorEx.class, "MIT");
        sClaw = hardwareMap.get(ServoImplEx.class, "sClaw");
        sDrone = hardwareMap.get(ServoImplEx.class, "sDrone");

        MEF.setDirection(DcMotorEx.Direction.REVERSE);
        MDT.setDirection(DcMotorEx.Direction.FORWARD);
        MEF.setDirection(DcMotorEx.Direction.REVERSE);
        MDF.setDirection(DcMotorEx.Direction.FORWARD);
        MLS.setDirection(DcMotorEx.Direction.REVERSE);
        MIT.setDirection(DcMotorEx.Direction.REVERSE);
        sClaw.setDirection(ServoImplEx.Direction.REVERSE);
        sDrone.setDirection(ServoImplEx.Direction.REVERSE);

        MEF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MDT.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MEF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MDF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MLS.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MIT.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        addSetpoint(10);
        sleep(1000);
        addSetpoint(-10);
    }

    public void addSetpoint(int SetPoint) {

        int SetPoint0 = MEF.getCurrentPosition() + (int) (SetPoint * FatorDeConversao);
        MEF.setTargetPosition(SetPoint0);
        MEF.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        MEF.setPower(1);

        int SetPoint1 = MDF.getCurrentPosition() + (int) (SetPoint * FatorDeConversao);
        MDF.setTargetPosition(SetPoint1);
        MDF.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        MDF.setPower(1);

        int SetPoint2 = MET.getCurrentPosition() + (int) (SetPoint * FatorDeConversao);
        MET.setTargetPosition(SetPoint2);
        MET.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        MET.setPower(1);

        int SetPoint3 = MDT.getCurrentPosition() + (int) (SetPoint * FatorDeConversao);
        MDT.setTargetPosition(SetPoint3);
        MDT.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        MDT.setPower(1);

        while (MEF.isBusy()) {
            telemetry.addData("Distancia percorrida", MEF.getCurrentPosition() / FatorDeConversao);
            telemetry.update();
        }
        while (MDF.isBusy()) {
            telemetry.addData("Distancia percorrida", MDF.getCurrentPosition() / FatorDeConversao);
            telemetry.update();
        }
        while (MET.isBusy()) {
            telemetry.addData("Distancia percorrida", MET.getCurrentPosition() / FatorDeConversao);
            telemetry.update();
        }
        while (MDT.isBusy()) {
            telemetry.addData("Distancia percorrida", MDT.getCurrentPosition() / FatorDeConversao);
            telemetry.update();
        }
        MEF.setPower(0);
        MEF.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        sleep(500);

        MDF.setPower(0);
        MDF.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        sleep(500);

        MET.setPower(0);
        MET.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        sleep(500);

        MDT.setPower(0);
        MDT.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        sleep(500);
    }
}