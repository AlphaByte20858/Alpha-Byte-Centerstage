//programação teleOP do Tanus
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.Name;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp (name = "testes", group = "OpMode")
public class testes extends OpMode {
    DcMotorEx MDFc, MDTc, METc, MEFc, MLS, MIN, MCL;
    Servo servoInDi, servoInEs, servoDrone, servoArm, servoClaw;
    IMU imu;
    ElapsedTime clawTime;

    double forward;
    double lateral;
    double rotation;
    double angle;
    int curPos;

    public void init(){

        MDFc = hardwareMap.get(DcMotorEx.class, "MDFc");
        MDTc = hardwareMap.get(DcMotorEx.class, "MDTc");
        MEFc = hardwareMap.get(DcMotorEx.class, "MEFc");
        METc = hardwareMap.get(DcMotorEx.class, "METc");

        MDFc.setDirection(DcMotorSimple.Direction.FORWARD);
        MDTc.setDirection(DcMotorSimple.Direction.FORWARD);
        MEFc.setDirection(DcMotorSimple.Direction.REVERSE);
        METc.setDirection(DcMotorSimple.Direction.REVERSE);

        MDFc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MDTc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MEFc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        METc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MDFc.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MDTc.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MEFc.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        METc.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        MLS = hardwareMap.get(DcMotorEx.class, "MLS");
        MIN = hardwareMap.get(DcMotorEx.class, "MIN");
        MCL = hardwareMap.get(DcMotorEx.class, "MCL");

        MLS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoInDi = hardwareMap.get(Servo.class, "servoInDi");
        servoInEs = hardwareMap.get(Servo.class, "servoInEs");
        servoDrone = hardwareMap.get(Servo.class, "servoDrone");
        servoArm = hardwareMap.get(Servo.class, "servoArm");
        servoClaw = hardwareMap.get(Servo.class, "servoClaw");

        imu = hardwareMap.get(IMU.class, "imu");

        servoDrone.setPosition(0.3);
        servoClaw.setPosition(0);
    }
    public void loop(){
        Movi();
        Arm();
        ServoClaw();
        Drone();
        Intake();
    }
    public void Movi(){
        forward = gamepad1.right_trigger - gamepad1.left_trigger;
        lateral = gamepad1.left_stick_x;
        rotation = gamepad1.right_stick_x * 0.75;

        orientedField(forward, lateral);

        double absPower = (Math.abs(lateral) + Math.abs(forward) + Math.abs(rotation));
        double denominador = Math.max(absPower, 1);

        double MEFf = (forward + lateral + rotation / denominador);
        double MDFf = (forward - lateral - rotation / denominador);
        double METf = (forward - lateral + rotation / denominador);
        double MDTf = (forward + lateral - rotation / denominador);

        if(gamepad1.right_bumper){
            moviPower(MEFf, MDFf , METf, MDTf  );
        }
        else {
            moviPower(MEFf * 0.8, MDFf * 0.8, METf * 0.8, MDTf * 0.8);
        }

        telemetry.addData("A velocidade do motorEsquerdoF é de:", MEFc.getVelocity() * 0.098);
        telemetry.addData("A velocidade do motorDireitoF é de:", MDFc.getVelocity() * 0.098);
        telemetry.addData("A velocidade do motorEsquerdoT é de:", METc.getVelocity() * 0.098);
        telemetry.addData("A velocidade do motorDireitoT é de:", MDTc.getVelocity() * 0.098);
    }
    public void orientedField(double pforward, double plateral){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        angle = orientation.getYaw(AngleUnit.RADIANS);
        forward = pforward * Math.cos(angle) - plateral * Math.sin(angle);
        rotation = pforward * Math.sin(angle) + plateral * Math.cos(angle);
    }
    public void moviPower(double paMEF, double paMDF, double paMET, double paMDT){
        MEFc.setPower(paMEF);
        MDFc.setPower(paMDF);
        METc.setPower(paMET);
        MDTc.setPower(paMDT);
    }
    public void Intake(){
        MIN.setPower((gamepad2.left_trigger - gamepad2.right_trigger) * 0.7);
    }
    public void Arm(){
        if (gamepad2.right_bumper) {
            MLS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MLS.setPower(0.6);
            MLS.getCurrentPosition();
        }
        else if (gamepad2.left_bumper) {
            MLS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MLS.setPower(-0.6);
            MLS.getCurrentPosition();
        }
        else{
            MLS.setTargetPosition(curPos);
            MLS.setPower(0.6);
            MLS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            setServoArm();
        }
    }
    public void setServoArm() {
        if (MLS.getCurrentPosition() > 500) {
            while (servoArm.getPosition() != 0.7) {
                servoArm.setPosition(0.7);
            }
        } else if (MLS.getCurrentPosition() < 500) {
            while (servoArm.getPosition() != 0) {
                servoArm.setPosition(0);
            }
        }
    }
    public void ServoClaw(){
        if (gamepad2.a && clawTime.seconds() > 0.4){
            if (servoClaw.getPosition() == 0)
                servoClaw.setPosition(0.3);
            else if (servoClaw.getPosition() == 0.3){
                servoClaw.setPosition(0.1);
            }
            else if (servoClaw.getPosition() == 0.3){
                servoClaw.setPosition(0);
            }
            clawTime.reset();
        }
    }
    public void Drone(){
        if (gamepad2.b){
            servoDrone.setPosition(0.6);
        }
    }
    public void climb(){
        if (gamepad2.dpad_up) {
            MCL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MCL.setPower(0.6);
            MCL.getCurrentPosition();
        }
        else if (gamepad2.dpad_down) {
            MCL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MCL.setPower(-0.6);
            MCL.getCurrentPosition();
        }
        else{
            MCL.setTargetPosition(curPos);
            MCL.setPower(0.6);
            MCL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
}