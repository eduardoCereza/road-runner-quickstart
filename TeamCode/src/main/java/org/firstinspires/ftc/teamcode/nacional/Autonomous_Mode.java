package org.firstinspires.ftc.teamcode.nacional;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "Teste RR", group = "Autonomous")
public class Autonomous_Mode extends LinearOpMode {


    //inicialização
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
                    left.setPower(0.8);
                    right.setPower(0.8);
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
                    left.setPower(-0.8);
                    right.setPower(-0.8);
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

    //TODO: VERIFICAR COMO FUNCIONAM OS SERVOS
    public class servos {
        private Servo garra, servoX, servoY;
        public servos(HardwareMap hardwareMap) {
            garra = hardwareMap.get(Servo.class, "garra");
            servoX = hardwareMap.get(Servo.class, "servoX");
            servoY = hardwareMap.get(Servo.class, "servoY");

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

    //TODO: CONTINUAR
    public class slide {
        private DcMotorEx gobilda;

        public slide(HardwareMap hardwareMap) {
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

        public class slideCloser implements Action {}
    }





    //ações




    @Override
    public void runOpMode(){

    }


}
