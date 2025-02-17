package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Periodo TeleOperado")
public class TeleOperado extends OpMode {

    // Motores de movimentação
    DcMotor leftF, leftB, rightF, rightB;

    DcMotorEx leftArm, rightArm;

    DcMotorEx gobildaMotor; // Declaração do motor
    boolean holdingPosition = false; // Variável que indica se o motor está segurando a posição
    final int MAX_POSITION = -4300; // Posição máxima permitida para o motor

    double x, y, turn;

    Servo servo, servo2, servo3;

    // Controle de movimentação
    double sin, theta, cos, power, max;
    double leftFPower, leftBPower, rightFPower, rightBPower;

    boolean modoAutoArm = false; // Começa no modo manual
    boolean lastPressR1 = false;
    boolean lastPressL1 = false;


    @Override
    public void init() {
        init_Hardware();
    }

    @Override
    public void loop() {

        // Variáveis para alternância de modo

        boolean pressR1 = gamepad2.dpad_up;
        boolean pressL1 = gamepad2.dpad_down;

        if (pressR1 && !lastPressR1) {
            modoAutoArm = true;
        }
        if (pressL1 && !lastPressL1) {
            modoAutoArm = false;
        }

        lastPressR1 = pressR1;
        lastPressL1 = pressL1;

        if (modoAutoArm) {
            telemetry.addLine("Modo Auto - Atuador");
        } else {
            telemetry.addLine("Modo Manual - Atuador");
            move_Slide();
            move_Base();
            //novo_Servo();
        }

        telemetry.update();

    }
    /*

    public void novo_Servo(){
        boolean leftTriggerPressed = gamepad2.left_trigger > 0.5;
        boolean rightTriggerPressed = gamepad2.right_trigger > 0.5;

        boolean rightPress = gamepad2.right_bumper;
        boolean leftPress = gamepad2.left_bumper;

        if (rightPress){
            servo2.setPosition(0.6);
        } else if (leftPress) {
            servo2.setPosition(1);
        }else {

        }

        if (leftTriggerPressed){
            servo3.setPosition(1);
        } else if (rightTriggerPressed) {
            servo3.setPosition(0);
        }else {

        }

        if (gamepad2.a) {
            servo.setPosition(0.5);
        } else if (gamepad2.b) {
            servo.setPosition(-0.7);
        } else {

        }
    }

     */
    public void init_Hardware(){

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


        telemetry.addLine("Sistema inicializado. Pronto para rodar!");
    }
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

    public void move_Base(){
        if (gamepad2.right_stick_y > 0.1){
            rightArm.setPower(0.5);
            leftArm.setPower(0.5);
        } else if (gamepad2.right_stick_y < 0) {
            rightArm.setPower(-0.5);
            leftArm.setPower(-0.5);
        }else{

        }
    }
}
