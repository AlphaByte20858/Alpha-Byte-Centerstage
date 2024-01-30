package org.firstinspires.ftc.teamcode;

 import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Autonomous (name = "Automonos teste", group = "LinearOpMode")
public class teste5 extends LinearOpMode {

    DcMotorEx MET, MDT, MEF, MDF, MLS, MIT = null;
    ServoImplEx sArm, sDrone = null;

    static final double COUNS_PER_MOTOR_REV = 560;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHELL_DIAMETER_INCHES = 1.96;
    double FatorDeConversao = (COUNS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHELL_DIAMETER_INCHES * Math.PI);

    public void runOpMode() {

        telemetry.addData("stats", "Initialize");
        telemetry.update();

        MET = hardwareMap.get(DcMotorEx.class, "MEF");
        MDT = hardwareMap.get(DcMotorEx.class, "MDF");
        MEF = hardwareMap.get(DcMotorEx.class, "MET");
        MDF = hardwareMap.get(DcMotorEx.class, "MDT");
        MLS = hardwareMap.get(DcMotorEx.class, "MLS");
        MIT = hardwareMap.get(DcMotorEx.class, "MIT");
        sArm = hardwareMap.get(ServoImplEx.class, "garra");
        sDrone = hardwareMap.get(ServoImplEx.class, "SD");

        MET.setDirection(DcMotorEx.Direction.REVERSE);
        MDT.setDirection(DcMotorEx.Direction.FORWARD);
        MEF.setDirection(DcMotorEx.Direction.REVERSE);
        MDF.setDirection(DcMotorEx.Direction.FORWARD);
        MLS.setDirection(DcMotorEx.Direction.REVERSE);
        MIT.setDirection(DcMotorEx.Direction.REVERSE);
        sArm.setDirection(ServoImplEx.Direction.REVERSE);
        sDrone.setDirection(ServoImplEx.Direction.REVERSE);

        MET.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
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

        int SetPoint0 = MET.getCurrentPosition() + (int) (SetPoint * FatorDeConversao);
        MET.setTargetPosition(SetPoint0);
        MET.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        MET.setPower(1);

        int SetPoint1 = MDT.getCurrentPosition() + (int) (SetPoint * FatorDeConversao);
        MDT.setTargetPosition(SetPoint1);
        MDT.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        MDT.setPower(1);

        int SetPoint2 = MEF.getCurrentPosition() + (int) (SetPoint * FatorDeConversao);
        MEF.setTargetPosition(SetPoint2);
        MEF.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        MEF.setPower(1);

        int SetPoint3 = MDF.getCurrentPosition() + (int) (SetPoint * FatorDeConversao);
        MDF.setTargetPosition(SetPoint3);
        MDF.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        MDF.setPower(1);

        while (MET.isBusy()) {
            telemetry.addData("Distancia percorrida", MET.getCurrentPosition() / FatorDeConversao);
            telemetry.update();
        }
        while (MDT.isBusy()) {
            telemetry.addData("Distancia percorrida", MDT.getCurrentPosition() / FatorDeConversao);
            telemetry.update();
        }
        while (MEF.isBusy()) {
            telemetry.addData("Distancia percorrida", MEF.getCurrentPosition() / FatorDeConversao);
            telemetry.update();
        }
        while (MDF.isBusy()) {
            telemetry.addData("Distancia percorrida", MDF.getCurrentPosition() / FatorDeConversao);
            telemetry.update();
        }
        MET.setPower(0);
        MET.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(500);

        MDT.setPower(0);
        MDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(500);

        MEF.setPower(0);
        MEF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(500);

        MDF.setPower(0);
        MDF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(500);
    }
}