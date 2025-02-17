package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Chassi_Move extends OpMode {

    // Motores de movimentação
    DcMotor leftF, leftB, rightF, rightB;
    // Variáveis do joystick
    double y, x, turn;
    // Controle de movimentação
    double sin, theta, cos, power, max;
    double leftFPower, leftBPower, rightFPower, rightBPower;
    @Override
    public void init() {
        // Configuração dos motores de movimentação
        leftF = hardwareMap.get(DcMotor.class, "leftf");
        leftB = hardwareMap.get(DcMotor.class, "leftb");
        rightF = hardwareMap.get(DcMotor.class, "rightf");
        rightB = hardwareMap.get(DcMotor.class, "rightb");

        leftF.setDirection(DcMotorSimple.Direction.REVERSE);
        leftB.setDirection(DcMotorSimple.Direction.REVERSE);

        leftF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addLine("Sistema inicializado. Pronto para rodar!");
        telemetry.update();
    }

    @Override
    public void loop() {

        y = -gamepad1.left_stick_y;
        x = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;


        theta = Math.atan2(y, x);
        power = Math.hypot(x, y);

        sin = Math.sin(theta - Math.PI / 4);
        cos = Math.cos(theta - Math.PI / 4);
        max = Math.max(Math.abs(sin), Math.abs(cos));

        leftFPower = power * cos / max + turn;
        rightFPower = power * sin / max - turn;
        leftBPower = power * sin / max + turn;
        rightBPower = power * cos / max - turn;

        if ((power + Math.abs(turn)) > 1) {
            leftFPower /= power + turn;
            leftBPower /= power + turn;
            rightFPower /= power + turn;
            rightBPower /= power + turn;
        }

        leftF.setPower(leftFPower);
        rightB.setPower(rightBPower);
        rightF.setPower(rightFPower);
        leftB.setPower(leftBPower);
    }
}