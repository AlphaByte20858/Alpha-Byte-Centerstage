package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.tensorflow.lite.task.vision.core.BaseVisionTaskApi;

@Autonomous (name = "Thiago", group = "LinearOpMode")
public class testeTH extends LinearOpMode {

    IMU imu;
    TFObjectDetector sla;

    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();
        imu = hardwareMap.get(IMU.class, "imu");
        sla = hardwareMap.get(TFObjectDetector.class, "sla");

        telemetry.addData("statucs", "inicializate");
        telemetry.update();

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        sla.activate();
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        waitForStart();
        runtime.reset();
    }
}

