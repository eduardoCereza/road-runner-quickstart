package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.nacional.PIDFController;

@TeleOp(name = "TeleOp Nacional", group = "Nacional")
public class TeleOp_Mode extends OpMode{

    //TODO: VARIÁVEIS DO CHASSI
    DcMotor leftF, leftB, rightF, rightB;
    double x, y, turn, power, theta, sin, cos, max,leftFPower, leftBPower, rightFPower, rightBPower;


    //TODO: VARIÁVEIS DO ATUADOR

    //Parte 1: Slide
    boolean holdingPosition = false;
    int MAX_POSITION = -4200;
    DcMotorEx slide;

    int encoderDegreesToAttain;

    //Parte 2: Base
    DcMotorEx leftArm, rightArm;

    //TODO: VARIÁVEIS DE MUDANÇA DE MODO
    boolean modoAutoArm = false, lastpress1 = false, lastpress2 = false;
    boolean modoAutoChassi = false, lastpress3 = false, lastpress4 = false;

    //TODO: IMPORTANDO CLASSES
    PIDFController controller;


    @Override
    public void init(){
        initHard();
    }

    @Override
    public void loop(){
        boolean preesArm1 = gamepad2.dpad_up;
        boolean preesArm2 = gamepad2.dpad_down;

        boolean pressChassi1 = gamepad1.dpad_up;
        boolean pressChassi2 = gamepad1.dpad_down;

        if (preesArm1 && !lastpress1){
            modoAutoArm = true;
        } else if (preesArm2 && !lastpress2) {
            modoAutoArm = false;
        }

        if(pressChassi1 && !lastpress3){
            modoAutoChassi = true;
        } else if (pressChassi2 && !lastpress4) {
            modoAutoChassi = false;
        }

        lastpress1 = preesArm1;
        lastpress2 = preesArm2;
        lastpress3 = pressChassi1;
        lastpress4 = pressChassi2;

        if (modoAutoArm){
            telemetry.addLine("Modo Auto - Atuador");
        } else {
            telemetry.addLine("Modo Manual - Atuador");
            move_Slide();
            move_Base();

        }

        if (modoAutoChassi){
            telemetry.addLine("Modo Auto - Chassi");
        } else {
            move_Chassi();
        }

        telemetry.update();


    }
    public void move_Slide(){
        int currentPosition = slide.getCurrentPosition(); // Obtém a posição atual do motor
        double joystickInput = gamepad2.left_stick_y; // Captura a entrada do joystick

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
    public void move_Base(){
        double minPower = 0;
        double maxPower = 0.3;
        controller = new PIDFController(1, 0, 0, 4);
        controller.setInputRange(-4000, 4000);
        controller.setSetPoint(encoderDegreesToAttain);
        controller.setOutputRange(minPower, maxPower);

        double input = -gamepad2.right_stick_y;

        double powerM = maxPower + controller.getComputedOutput(leftArm.getCurrentPosition());
        double powerS = minPower - controller.getComputedOutput(leftArm.getCurrentPosition());


        double powerM1 = maxPower + controller.getComputedOutput(rightArm.getCurrentPosition());
        double powerS1 = minPower - controller.getComputedOutput(rightArm.getCurrentPosition());


        if (input > 0){
            leftArm.setPower(powerM);
            rightArm.setPower(powerM1);
        } else if(input < 0) {
            leftArm.setPower(powerS);
            rightArm.setPower(powerS1);
        }else {
            leftArm.setPower(0);
            rightArm.setPower(0);
        }


    }
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
    public void initHard(){

        leftArm = hardwareMap.get(DcMotorEx.class, "left");
        rightArm = hardwareMap.get(DcMotorEx.class, "right");

        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
    }

}
