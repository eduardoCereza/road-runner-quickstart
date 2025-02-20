package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.nacional.PIDFController;

@TeleOp
public class Mover_Arm extends OpMode {
    DcMotor leftF, leftB, rightF, rightB;
    DcMotorEx leftArm, rightArm, gobildaMotor;
    boolean lastPressR1 = false;
    boolean lastPressL1 = false;

    boolean modoAutoArm = false;

    boolean holdingPosition = false;
    final int MAX_POSITION = -4300;

    int encoderDegreesToAttain;
    private PIDFController controller;

    @Override
    public void init() {
        initHardware();

    }

    @Override
    public void loop() {

        double minPower = 0.01;
        double maxPower = 0.3;
        controller = new PIDFController(1, 0, 0, 4);
        controller.setInputRange(50, 4000);

        controller.setSetPoint(encoderDegreesToAttain);
        controller.setOutputRange(minPower, maxPower);

        double powerM = minPower + controller.getComputedOutput(leftArm.getCurrentPosition());
        double powerS = minPower - controller.getComputedOutput(leftArm.getCurrentPosition());

        double powerM1 = minPower + controller.getComputedOutput(rightArm.getCurrentPosition());
        double powerS1 = minPower - controller.getComputedOutput(rightArm.getCurrentPosition());

        if (gamepad2.right_stick_y < 0){
            leftArm.setPower(powerM);
            rightArm.setPower(powerM1);
        } else if((gamepad2.right_stick_y > 0)) {
            leftArm.setPower(powerS);
            rightArm.setPower(powerS1);
        }else {
            leftArm.setPower(0);
            rightArm.setPower(0);
        }

        move_Slide();

        telemetry.addData("Posição Left", leftArm.getCurrentPosition());
        telemetry.update();
    }
    public void initHardware(){

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

        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        gobildaMotor = hardwareMap.get(DcMotorEx.class, "gobilda"); // Mapeia o motor do hardware
        gobildaMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Define o comportamento de freio ao parar
        gobildaMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reseta os encoders
        gobildaMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Configura o motor para usar encoders
    }
    /*public void move_Slide(){
        int currentPosition = gobildaMotor.getCurrentPosition(); // Obtém a posição atual do motor
        double joystickInput = gamepad2.left_stick_y; // Captura a entrada do joystick

        // Se o joystick for movido para cima e a posição for menor que 0, move o motor
        if (joystickInput > 0 && currentPosition < 0) {
            gobildaMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            gobildaMotor.setPower(joystickInput);
            holdingPosition = false; // O motor está se movendo, então não está segurando posição
        }
        // Se o joystick for movido para baixo e ainda não atingiu o limite, move o motor
        else if (joystickInput < 0 && currentPosition > MAX_POSITION) {
            gobildaMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            gobildaMotor.setPower(joystickInput);
            holdingPosition = false; // O motor está se movendo, então não está segurando posição
        }
        // Se o joystick estiver parado e o motor ainda não estiver segurando a posição
        else if (!holdingPosition) { // O operador ! (negação) verifica se holdingPosition é false
            gobildaMotor.setTargetPosition(currentPosition); // Define a posição atual como alvo
            gobildaMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Mantém o motor na posição
            gobildaMotor.setPower(0.1); // Aplica uma pequena potência para segurar a posição
            holdingPosition = true; // Marca que o motor está segurando a posição
        }

        telemetry.addData("Joystick:", joystickInput);
        telemetry.addData("Posição Atual:", currentPosition);
        telemetry.update();
    }

     */

    public void move_Slide(){
        int currentPosition = gobildaMotor.getCurrentPosition(); // Obtém a posição atual do motor
        double joystickInput = gamepad2.left_stick_y; // Captura a entrada do joystick

        // Se o joystick for movido para cima e a posição for menor que 0, move o motor
        if (joystickInput > 0 && currentPosition < 0) {
            gobildaMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            gobildaMotor.setPower(joystickInput);
            holdingPosition = false; // O motor está se movendo, então não está segurando posição
        }
        // Se o joystick for movido para baixo e ainda não atingiu o limite, move o motor
        else if (joystickInput < 0 && currentPosition > MAX_POSITION) {
            gobildaMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            gobildaMotor.setPower(joystickInput);
            holdingPosition = false; // O motor está se movendo, então não está segurando posição
        }
        // Se o joystick estiver parado e o motor ainda não estiver segurando a posição
        else if (!holdingPosition) { // O operador ! (negação) verifica se holdingPosition é false
            gobildaMotor.setTargetPosition(currentPosition); // Define a posição atual como alvo
            gobildaMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Mantém o motor na posição
            gobildaMotor.setPower(0.1); // Aplica uma pequena potência para segurar a posição
            holdingPosition = true; // Marca que o motor está segurando a posição
        }

        telemetry.addData("Joystick:", joystickInput);
        telemetry.addData("Posição Atual:", currentPosition);
        telemetry.update();
    }


}
