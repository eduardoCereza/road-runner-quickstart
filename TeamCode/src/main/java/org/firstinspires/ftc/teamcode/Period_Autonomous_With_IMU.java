package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.nacional.PController;

@Autonomous
public class Period_Autonomous_With_IMU extends LinearOpMode {

    DcMotorEx leftF, leftB, rightF, rightB;
    IMU imu;
    private DcMotorEx rex, rightArm, leftArm;
    private CRServo intake1, intake2;

    private Move_IMU moveCM;
    private Strafe_IMU strafe;

    private Gyro gyro;

    public void runOpMode(){

        leftF = hardwareMap.get(DcMotorEx.class, "leftf");
        leftB = hardwareMap.get(DcMotorEx.class, "leftb");
        rightF = hardwareMap.get(DcMotorEx.class, "rightf");
        rightB = hardwareMap.get(DcMotorEx.class, "rightb");

        leftArm = hardwareMap.get(DcMotorEx.class, "left");
        rightArm = hardwareMap.get(DcMotorEx.class, "right");

        leftF.setDirection(DcMotorEx.Direction.REVERSE);
        leftB.setDirection(DcMotorEx.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");

        leftF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        moveCM = new Move_IMU();
        strafe = new Strafe_IMU();
        gyro = new Gyro();

        waitForStart();

        if (opModeIsActive()){

            strafe.move(100, leftF, leftB, rightF, rightB, imu);


        }

    }


}
