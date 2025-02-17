package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Autonomous_Mode extends LinearOpMode {

    DcMotor armMotor1;

    Arm_Move armMove = new Arm_Move();

    @Override
    public void runOpMode(){

        armMotor1 = hardwareMap.get(DcMotor.class, "armMotor1");

        armMove.move_Arm(armMotor1, 1000);

    }


}
