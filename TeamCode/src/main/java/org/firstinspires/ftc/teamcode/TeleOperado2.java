package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TeleOperado2 extends OpMode {
    DcMotor leftF, leftB, rightF, rightB;
    DcMotorEx leftArm, rightArm, gobildaMotor;
    boolean holdingPosition = false;
    final int MAX_POSITION = -4300;
    double x, y, turn;
    Servo servo, servo2, servo3;
    double sin, theta, cos, power, max;
    double leftFPower, leftBPower, rightFPower, rightBPower;
    boolean modoAutoArm = false;
    boolean lastPressR1 = false;
    boolean lastPressL1 = false;

    @Override
    public void init() {
        initHardware();
    }

    @Override
    public void loop() {
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

        move_Chassi();

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

    public void initHardware(){

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

        /*
        servo = hardwareMap.get(Servo.class, "servo");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo3 = hardwareMap.get(Servo.class, "servo3");

        servo.setDirection(Servo.Direction.REVERSE);

         */

        leftArm = hardwareMap.get(DcMotorEx.class, "left");
        rightArm = hardwareMap.get(DcMotorEx.class, "right");


        gobildaMotor = hardwareMap.get(DcMotorEx.class, "gobilda"); // Mapeia o motor do hardware
        gobildaMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Define o comportamento de freio ao parar
        gobildaMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reseta os encoders
        gobildaMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Configura o motor para usar encoders
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
        /*
        double input = gamepad2.right_stick_y;
        if (input > 0.1){
            rightArm.setPower(input);
            leftArm.setPower(input);
        } else if (input < 0) {
            rightArm.setPower(input);
            leftArm.setPower(input);
        }else{
            rightArm.setPower(0);
            leftArm.setPower(0);
        }

 */}
    public void move_Chassi(){
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
