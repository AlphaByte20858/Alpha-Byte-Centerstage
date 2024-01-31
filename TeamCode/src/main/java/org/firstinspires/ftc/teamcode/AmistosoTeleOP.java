/*esta programação será utilizada no amistoso da UnderCtrl;
nesta programação não haverá incluso sistema de climb, pois estará sendo trabalhada melhor após o
evento para sua melhor funcionalidade*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp (name = "AmistosoTeleOP", group = "OpMode")
public class AmistosoTeleOP extends OpMode {

    DcMotorEx MEF, MDF, MET, MDT, MCL, MIT, MLS;
    IMU imu;
    Servo sArm, sDrone, sClaw;
    Boolean isDroneOpen = true;
    Boolean clawOpen = false;
    ElapsedTime garraTime = new ElapsedTime();
    ElapsedTime droneTime = new ElapsedTime();
    ElapsedTime servoTime = new ElapsedTime();
    public void init() {

        MEF = hardwareMap.get(DcMotorEx.class, "MEF");
        MDF = hardwareMap.get(DcMotorEx.class, "MDF");
        MET = hardwareMap.get(DcMotorEx.class, "MET");
        MDT = hardwareMap.get(DcMotorEx.class, "MDT");
        MLS = hardwareMap.get(DcMotorEx.class, "MLS");
        MCL = hardwareMap.get(DcMotorEx.class, "MCL");
        MIT = hardwareMap.get(DcMotorEx.class, "MIT");
        sArm = hardwareMap.get(Servo.class, "sArm");
        sDrone = hardwareMap.get(Servo.class, "sDrone");
        sClaw = hardwareMap.get(Servo.class, "sClaw");

        MEF.setDirection(DcMotorEx.Direction.REVERSE);
        MDF.setDirection(DcMotorEx.Direction.FORWARD);
        MET.setDirection(DcMotorEx.Direction.REVERSE);
        MDT.setDirection(DcMotorEx.Direction.FORWARD);

        MLS.setDirection(DcMotorEx.Direction.REVERSE);
        MCL.setDirection(DcMotorEx.Direction.REVERSE);
        MCL.setDirection(DcMotorEx.Direction.REVERSE);
        sArm.setDirection(Servo.Direction.REVERSE);
        sDrone.setDirection(Servo.Direction.FORWARD);
        sClaw.setDirection(Servo.Direction.FORWARD);

        modemoto(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        modemoto(DcMotorEx.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "IMU");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        servoTime.startTime();
        droneTime.startTime();
        garraTime.startTime();

        MLS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sArm.setPosition(0);
    }

    @Override
    public void loop() {
        move();
        RAW();
        Flinear();
        drone();
        ITK();
    }

    public void move() {

        // Criação das váriaveis de força para a movimentação

        double axial, lateral, yaw;

        axial = gamepad1.left_trigger - gamepad1.right_trigger;
        lateral = gamepad1.left_stick_x;
        yaw = gamepad1.right_stick_x * 0.7;

        // Criação de váriaveis par8 a movimentação em 8 direções
        double denominador = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);
        YawPitchRollAngles XYZangles = imu.getRobotYawPitchRollAngles();
        AngularVelocity XYZvelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);


        // Calculos para a movimentação das rodas omnidirectionais mecanum
        double MEFp = (axial + lateral + yaw / denominador);
        double MDFp = (axial - lateral - yaw / denominador);
        double METp = (axial - lateral + yaw / denominador);
        double MDTp = (axial + lateral - yaw / denominador);

        allMotorsPower(-MEFp * 0.7, -MDFp * 0.7, METp * 0.7, MDTp * 0.7);
    }

    public void allMotorsPower(double paMEF, double paMDF, double paMET, double paMDT) {
        MEF.setPower(paMEF);
        MDF.setPower(paMDF);
        MET.setPower(paMET);
        MDT.setPower(paMDT);
    }
    public void modemoto(DcMotor.RunMode mode) {
        MEF.setMode(mode);
        MDF.setMode(mode);
        MET.setMode(mode);
        MDT.setMode(mode);
    }

    //sistema linear
    public void Flinear() {
        double powerLinear = 0.6;

        if (gamepad2.right_bumper) {
            MLS.setPower(powerLinear);
        }
        else if (gamepad2.left_bumper) {
            MLS.setPower(-powerLinear);
        }
        else {
            MLS.setPower(0);
        }
    }

    //garra
    public void RAW(){
        if (garraTime.seconds() > 0.5)
            if (gamepad2.a && clawOpen == false ){
                sArm.setPosition(0.5);
                clawOpen = true;
            }

            else if (gamepad2.a && clawOpen == true ){
                sArm.setPosition(0);
                clawOpen = false;
            }
            garraTime.reset();
    }

    //sistema de lançamento do avião
    public void drone(){
        if (gamepad2.y && isDroneOpen == true && droneTime.seconds() > 2){
            sDrone.setPosition(0);
            isDroneOpen = false;
            droneTime.reset();
        }
        if (gamepad2.y && isDroneOpen == false && droneTime.seconds() > 2){
            sDrone.setPosition(0.5);
            isDroneOpen = true;
            droneTime.reset();
        }
    }

    public void ITK() {
        MIT.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
    }
}