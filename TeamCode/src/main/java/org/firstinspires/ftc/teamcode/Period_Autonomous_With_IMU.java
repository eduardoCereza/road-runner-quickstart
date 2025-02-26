package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
@Config
@Autonomous
public class Period_Autonomous_With_IMU extends LinearOpMode{

    Move_IMU move;
    Strafe_IMU strafe;

    DcMotorEx leftF, leftB, rightF, rightB;

    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException{

        move = new Move_IMU();
        strafe = new Strafe_IMU();

        leftF = hardwareMap.get(DcMotorEx.class, "leftf");
        leftB = hardwareMap.get(DcMotorEx.class, "leftb");
        rightF = hardwareMap.get(DcMotorEx.class, "rightf");
        rightB = hardwareMap.get(DcMotorEx.class, "rightb");

        imu = hardwareMap.get(IMU.class, "imu");

        leftF.setDirection(DcMotorSimple.Direction.REVERSE);
        leftB.setDirection(DcMotorSimple.Direction.REVERSE);

        leftF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        while(opModeIsActive()){

            move.moveForwardStraight(leftF, leftB, rightF, rightB, imu, -50, 300);

            sleep(1000);

            strafe.moveStrafe(leftF, leftB, rightF, rightB, imu, 50, 300);

            break;

        }
    }
}