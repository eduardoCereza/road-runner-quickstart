package org.firstinspires.ftc.teamcode.nacional;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Autonomous_Mode extends LinearOpMode {

    DcMotor leftArm, rightArm;

    Arm_Move armMove = new Arm_Move();

    @Override
    public void runOpMode(){

        leftArm = hardwareMap.get(DcMotor.class, "left");

        armMove.move_Arm(leftArm, 1000);

    }


}
