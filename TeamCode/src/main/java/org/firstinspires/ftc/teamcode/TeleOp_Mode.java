package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.nacional.PIDFController;

@TeleOp(name = "TeleOp Nacional", group = "Nacional")
public class TeleOp_Mode extends OpMode{

    int encoderPoint;

    //TODO: VARIÁVEIS DO CHASSI
    //Definir variáveis dos motores
    DcMotor leftF, leftB, rightF, rightB;
    //Definir variáveis para controle do CHASSI
    double x, y, turn, power, theta, sin, cos, max,leftFPower, leftBPower, rightFPower, rightBPower;


    //TODO: VARIÁVEIS DO ATUADOR

    //Parte 1: Slide
    boolean holdingPosition = false; ///Definir
    boolean holdingPosition2 = false;
    boolean holdingPosition3 = false;
    int MAX_POSITION = -3160, MAXLIMITL = 700, MAXLIMITR = 700; ///Posição máxima do SLIDE
    DcMotorEx slide; ///Definir variável do SLIDE

    Servo servo, servo3; ///Definir variáveis do SERVO

    //Parte 2: Base
    DcMotorEx leftArm, rightArm; ///Definir variáveis do motor da BASE

    //TODO: VARIÁVEIS DE MUDANÇA DE MODO
    boolean modoAutoArm = false, lastpress1 = false, lastpress2 = false; ///Variáveis para alternar entre modo automático e manual do ATUADOR

    //TODO: IMPORTANDO CLASSES
    PIDFController controller; ///Importando classe do PIDF


    @Override
    public void init(){
        ///Inicialização dos hardware a partir do método
        initHard();
    }

    @Override
    public void loop(){
        ///Variáveis que acionam o modos auto e manual, tanto do CHASSI quanto do ATUADOR
        boolean preesArm1 = gamepad2.dpad_up;
        boolean preesArm2 = gamepad2.dpad_down;

        ///Condição que permite acessar os modos auto e manual, tanto do CHASSI quanto do ATUADOR
        if (preesArm1 && !lastpress1){
            modoAutoArm = true;
        } else if (preesArm2 && !lastpress2) {
            modoAutoArm = false;
        }

        lastpress1 = preesArm1;
        lastpress2 = preesArm2;


        ///Dependendo do modo acionado, o robô vai fazer movimentos do modo auto ou manual
        if (modoAutoArm){
            telemetry.addLine("Modo Auto - Atuador");
        } else {
            telemetry.addLine("Modo Manual - Atuador");
            move_Slide();
            move_Base();
            move_Servo();
        }


        move_Chassi();


        telemetry.update();
    }

    //Método para mover o SLIDE
    public void move_Slide(){
        int currentPosition = slide.getCurrentPosition(); // Obtém a posição atual do motor
        double joystickInput = gamepad2.left_stick_y; // Captura a entrant do joystick

        // Se o joystick for movido para cima e a posição for menor que 0, move o motor
        if (joystickInput > 0 && currentPosition < 0) {
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.setPower(joystickInput);
            holdingPosition = false; // O motor está se movendo, então não está segurando posição
        }
        // Se o joystick for movido para baixo e ainda não atingiu o limite, move o motor
        else if (joystickInput < 0 && currentPosition > MAX_POSITION) {
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.setPower(joystickInput);
            holdingPosition = false; // O motor está se movendo, então não está segurando posição
        }
        // Se o joystick estiver parado e o motor ainda não estiver segurando a posição
        else if (!holdingPosition) { // O operador ! (negação) verifica se holdingPosition é false
            slide.setTargetPosition(currentPosition); // Define a posição atual como alvo
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Mantém o motor na posição
            slide.setPower(0.1); // Aplica uma pequena potência para segurar a posição
            holdingPosition = true; // Marca que o motor está segurando a posição
        }

    }
    //Método para mover a base do Atuador
    public void move_Base(){
        int currentPosition2 = leftArm.getCurrentPosition();
        int currentPosition3 = rightArm.getCurrentPosition();
        double minPower = 0.2;
        double maxPower = 0.2;
        controller = new PIDFController(10, 3, 4, 12);
        controller.setInputRange(-4000, 4000);
        controller.setSetPoint(encoderPoint);
        controller.setOutputRange(minPower, maxPower);

        double input = -gamepad2.right_stick_y;

        double powerM = maxPower + controller.getComputedOutput(leftArm.getCurrentPosition());
        double powerM1 = maxPower + controller.getComputedOutput(rightArm.getCurrentPosition());


        //double powerS = maxPower - controller.getComputedOutput(leftArm.getCurrentPosition());
       // double powerS1 = maxPower - controller.getComputedOutput(rightArm.getCurrentPosition());


        if(currentPosition2 < MAXLIMITL && currentPosition3 < MAXLIMITR) {
            if (input > 0) {
                leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                leftArm.setPower(powerM/1.5);
                rightArm.setPower(powerM1/1.5);

                holdingPosition2 = false;
            } else if (input < 0) {
                leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                leftArm.setPower(-powerM / 3);
                rightArm.setPower(-powerM1 / 3);

                holdingPosition2 = false;
            } else if (!holdingPosition2) {

                leftArm.setTargetPosition(currentPosition2);
                leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftArm.setPower(0);
                leftArm.setPower(powerM*2);

                rightArm.setTargetPosition(currentPosition3);
                rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightArm.setPower(0);
                rightArm.setPower(powerM1*2);

                holdingPosition2 = true;

            }
        }else {
            if (input < 0) {
                leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                leftArm.setPower(-powerM / 3);
                rightArm.setPower(-powerM1 / 3);

                holdingPosition3 = false;
            } else if (!holdingPosition3) {
                leftArm.setTargetPosition(currentPosition2);
                leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftArm.setPower(0);
                leftArm.setPower(powerM*2);

                rightArm.setTargetPosition(currentPosition3);
                rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightArm.setPower(0);
                rightArm.setPower(powerM1*2);

                holdingPosition3 = true;

            }
        }



        telemetry.addData("Posição LEFT: ", currentPosition2);
        telemetry.addData("Posição RIGHT: ", currentPosition3);

    }
    //Método para mover o CHASSI do robô
    public void move_Chassi(){
        y = gamepad1.left_stick_y;
        x = -gamepad1.left_stick_x;
        turn = -gamepad1.right_stick_x;


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
    //Método para inicializar os hardwares
    public void initHard(){

        leftArm = hardwareMap.get(DcMotorEx.class, "left");
        rightArm = hardwareMap.get(DcMotorEx.class, "right");

        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightArm.setDirection(DcMotorSimple.Direction.REVERSE);
        leftArm.setDirection(DcMotorSimple.Direction.REVERSE);

        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide = hardwareMap.get(DcMotorEx.class, "gobilda"); // Mapeia o motor do hardware
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Define o comportamento de freio ao parar
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reseta os encoders
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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


        servo = hardwareMap.get(Servo.class, "servo");
        servo3 = hardwareMap.get(Servo.class, "servo3");

        servo.setDirection(Servo.Direction.REVERSE);

    }
    //Método para mover os Servos
    public void move_Servo(){

        if(gamepad2.right_bumper){
            servo3.setPosition(0.3);
            servo.setPosition(0.5);
        }else {
            servo3.setPosition(0);
            servo.setPosition(0);
        }
    }

}
