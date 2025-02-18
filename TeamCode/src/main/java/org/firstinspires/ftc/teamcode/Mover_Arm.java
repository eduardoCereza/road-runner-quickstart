package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class Mover_Arm extends OpMode {

    DcMotorEx leftMotor, rightMotor; // Declaração dos dois motores
    boolean holdingPosition2 = false; // Variável que indica se os motores estão segurando a posição
    final int MAX_POSITION = -4300; // Posição máxima permitida para os motores

    @Override
    public void init() {
        leftMotor = hardwareMap.get(DcMotorEx.class, "left"); // Mapeia o motor esquerdo
        rightMotor = hardwareMap.get(DcMotorEx.class, "right"); // Mapeia o motor direito

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        int currentPosition2= leftMotor.getCurrentPosition(); // Assume que ambos os motores têm a mesma posição
        double joystickInput = gamepad2.right_stick_y; // Captura a entrada do joystick

        // Se o joystick for movido para cima e a posição for menor que 0, move os motores
        if (joystickInput > 0 && currentPosition2 < 0) {
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotor.setPower(joystickInput);
            rightMotor.setPower(joystickInput);
            holdingPosition2 = false; // Os motores estão se movendo, então não estão segurando posição
        }
        // Se o joystick for movido para baixo e ainda não atingiu o limite, move os motores
        else if (joystickInput < 0 && currentPosition2 > MAX_POSITION) {
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotor.setPower(joystickInput);
            rightMotor.setPower(joystickInput);
            holdingPosition2 = false; // Os motores estão se movendo, então não estão segurando posição
        }
        // Se o joystick estiver parado e os motores ainda não estiverem segurando a posição
        else if (!holdingPosition2) { // O operador ! (negação) verifica se holdingPosition é false
            leftMotor.setTargetPosition(currentPosition2);
            rightMotor.setTargetPosition(currentPosition2);
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotor.setPower(0.1);
            rightMotor.setPower(0.1);
            holdingPosition2 = true; // Marca que os motores estão segurando a posição
        }

        telemetry.addData("Posição Atual:", currentPosition2);
        telemetry.update();
    }
}
