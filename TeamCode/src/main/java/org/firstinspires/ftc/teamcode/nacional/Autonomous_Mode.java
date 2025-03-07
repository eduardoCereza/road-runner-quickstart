package org.firstinspires.ftc.teamcode.nacional;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "Teste RR", group = "Autonomous")
    public class Autonomous_Mode extends LinearOpMode {

    //Initilização e criação de movimentos dos componentes do atuador
    public class leftandright {
        private DcMotorEx left, right;
        public leftandright(HardwareMap hardwareMap){
            left = hardwareMap.get(DcMotorEx.class, "left");
            right = hardwareMap.get(DcMotorEx.class, "right");

            left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        //ação de ir para cima
        public class leftup implements Action{
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    left.setPower(1.0);
                    right.setPower(1.0);
                    initialized = true;
                }
                double pos = left.getCurrentPosition();
                packet.put("leftPos", pos);
                //TODO: VERIFICAR
                if(pos < 300.0){
                    return  true;
                }
                else {
                    left.setPower(0);
                    return false;
                }

            }
        }

        //criando a ação para ser mais conveniente
        public Action leftup() {
            return new leftup();
        }


        //ação de ir para baixo
        public class leftdown implements Action{
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    left.setPower(-0.7);
                    right.setPower(-0.7);
                    initialized = true;
                }
                double pos = left.getCurrentPosition();
                packet.put("leftPos", pos);
                //TODO: VERIFICAR
                if(pos > 300.0){
                    return  true;
                }
                else {
                    left.setPower(0);
                    return false;
                }

            }
        }
        //criando a ação para ser mais conveniente
        public Action leftdown() {
            return new leftdown();
        }

    }

    //TODO: VERIFICAR COMO FUNCIONAM OS SERVOS PARA ACERTAR OS MOVIMENTOS
    public class servos {
        private Servo garra, servoX, servoY;
        public servos(HardwareMap hardwareMap) {
            garra = hardwareMap.get(Servo.class, "servo");
            servoY = hardwareMap.get(Servo.class, "servo3");

        }
        //ação de abrir garra
        public class GarraOpen implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                // TODO: VERIFICAR
                garra.setPosition(1.0);
                return false;
            }
        }
        //ação de abrir a garra
        public Action GarraOpen(){
            return new GarraOpen();
        }

        public class GarraClosed implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // TODO: VERIFICAR
                garra.setPosition(0.1);
                return false;
            }
        }
        public Action GarraClosed(){
            return new GarraClosed();
        }

        public class servoYUP implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // TODO: VERIFICAR
                servoY.setPosition(1.0);
                return false;
            }
        }
        public Action servoYUP() {
            return new servoYUP();
        }

    }

    //TODO: VERIFICAR OS ENCODERS DO GOBILDA PARA AJUSTES
    public class Slide {
        private DcMotorEx gobilda;
        public Slide(HardwareMap hardwareMap) {
            gobilda = hardwareMap.get(DcMotorEx.class, "gobilda");
            gobilda.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        public class slideExtender implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if (!initialized) {
                    gobilda.setPower(1.0);
                    initialized = true;
                }
                double pos = gobilda.getCurrentPosition();
                packet.put("slidePos", pos);
                //TODO: VERIFICAR
                if(pos < 300.0) {
                    return true;
                }
                else {
                    gobilda.setPower(0);
                    return false;
                }

            }

        }
        public Action slideExtender(){
            return new slideExtender();
        }
        public class slideCloser implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    gobilda.setPower(-1.0);
                    initialized = true;
                }
                double pos = gobilda.getCurrentPosition();
                packet.put("slidePos", pos);
                //TODO: VERIFICAR
                if (pos > 300.0) {
                    return true;
                } else {
                    gobilda.setPower(0);
                    return false;
                }
            }
        }
        public Action slideCloser(){
            return new slideCloser();
        }
    }

    @Override
    public void runOpMode(){
        //inicializa o chassi em uma posição especifica
        //TODO: VERIFICAR ESSA TAL POSIÇÂO
        //Make sure your MecanumDrive is instantiated at the correct pose. If you end up using lineToX(), lineToY(),
        //strafeTo(), splineTo(), or any of their variants in your code, if the initial pose is wrong,
        // all future movements will be thrown off.
        Pose2d initialPose = new Pose2d(0, 0,Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        //cria uma instância para as o slide
        Slide slide = new Slide(hardwareMap);
        //cria uma instância para os motores
        leftandright motors = new leftandright(hardwareMap);
        //cria uma instância para os servos;
        servos servos = new servos(hardwareMap);

        //acho que isso é só para utilizar camera ent se pá vai ter q tirar
        int visionOutputPosition = 1;

        //TODO: A PARTIR DAQUI QUE È POSSÍVEL COLOCAR NOSSAS TRAJETÓRIAS
        //TODO: ESTUDAR OQUE CADA EXPRESSÂO SIGNIFICA EXEMPLO: LINETOYSPLINEHEADING
        TrajectoryActionBuilder trajectoryA = drive.actionBuilder(initialPose)
                .lineToX(10);


        TrajectoryActionBuilder trajectoryB = drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(90.00)))
                .lineToY(20);

        TrajectoryActionBuilder trajectoryC = drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(90.00)))
                .lineToY(30);

        TrajectoryActionBuilder trajectory0 = drive.actionBuilder(new Pose2d(-0.69, -67.66, Math.toRadians(90.00)))
                .splineTo(new Vector2d(29.94, -46.17), Math.toRadians(35.05))
                .splineTo(new Vector2d(48.69, -49.83), Math.toRadians(-11.04));

        //ações que acontecem ao inicializar
        Actions.runBlocking(servos.GarraOpen());

        while(!isStopRequested() && !opModeIsActive()){
            int position = visionOutputPosition;
            telemetry.addData("Position durint INIT", position);
            telemetry.update();
        }
        int startPosition = visionOutputPosition;
        telemetry.addData("Startinng Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = trajectory0.build();
        } else if (startPosition == 2) {
            trajectoryActionChosen = trajectoryB.build();
        }else {
            trajectoryActionChosen = trajectoryC.build();
        }
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen
                )
        );
    }
}