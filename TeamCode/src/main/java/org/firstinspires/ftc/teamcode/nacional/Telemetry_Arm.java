package org.firstinspires.ftc.teamcode.nacional;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class Telemetry_Arm extends OpMode {
    DcMotorEx slide, leftArm, rightArm;

    public void init(){
        slide = hardwareMap.get(DcMotorEx.class, "gobilda"); // Mapeia o motor do hardware
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Define o comportamento de freio ao parar
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reseta os encoders
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftArm = hardwareMap.get(DcMotorEx.class, "left");
        rightArm = hardwareMap.get(DcMotorEx.class, "right");

        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    public void loop(){

        telemetry.addData("Posição", slide.getCurrentPosition());
        telemetry.addData("Posição 2: ", leftArm.getCurrentPosition());
        telemetry.addData("Posição 3: ", rightArm.getCurrentPosition());

        telemetry.update();
    }
}
