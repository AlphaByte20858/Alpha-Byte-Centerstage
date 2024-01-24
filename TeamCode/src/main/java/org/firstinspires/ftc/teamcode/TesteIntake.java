package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Intake teste", group= "OpMode")
public class TesteIntake extends OpMode {
    DcMotor MIN;
    @Override
    public void init() {
        MIN = hardwareMap.get(DcMotor.class, "Linear");
        MIN.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        MIN.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
    }
}
