package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Init_Hardware {
 public void init(DcMotor leftF, DcMotor rightF, DcMotor leftB, DcMotor rightB, DcMotorEx leftArm, DcMotorEx rightArm, DcMotorEx gobildaMotor, HardwareMap hardwareMap) {

     leftF = hardwareMap.get(DcMotor.class, "leftf");
     leftB = hardwareMap.get(DcMotor.class, "leftb");
     rightF = hardwareMap.get(DcMotor.class, "rightf");
     rightB = hardwareMap.get(DcMotor.class, "rightb");

     leftF.setDirection(DcMotorSimple.Direction.REVERSE);
     leftB.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
        servo = hardwareMap.get(Servo.class, "servo");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo3 = hardwareMap.get(Servo.class, "servo3");

        servo.setDirection(Servo.Direction.REVERSE);

         */

     leftArm = hardwareMap.get(DcMotorEx.class, "left");
     rightArm = hardwareMap.get(DcMotorEx.class, "right");

     rightArm.setDirection(DcMotorSimple.Direction.REVERSE);

     gobildaMotor = hardwareMap.get(DcMotorEx.class, "gobilda"); // Mapeia o motor do hardware
     gobildaMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Define o comportamento de freio ao parar
     gobildaMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reseta os encoders
     gobildaMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Configura o motor para usar encoders
 }
}
