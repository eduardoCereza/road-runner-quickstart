package org.firstinspires.ftc.teamcode.nacional;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class Arm_PIDF extends LinearOpMode {

    DcMotor motorArm, motorArm2;

    @Override
    public void runOpMode() throws InterruptedException{

        int encoderDegreesToAttain = 1500;
        double minPower = 0.01;
        double maxPower = 0.5;
        PIDFController pidfController = new PIDFController(0.1, 0, 0, 2);
        pidfController.setInputRange(50, 4000);
        pidfController.setSetPoint(encoderDegreesToAttain);
        pidfController.setOutputRange(minPower, maxPower);

        motorArm = hardwareMap.get(DcMotor.class, "left");
        motorArm2 = hardwareMap.get(DcMotor.class, "right");

        motorArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        double powerM = minPower + pidfController.getComputedOutput(motorArm.getCurrentPosition());
        double powerS = minPower - pidfController.getComputedOutput(motorArm.getCurrentPosition());

        double powerM2 = minPower + pidfController.getComputedOutput(motorArm2.getCurrentPosition());
        double powerS2 = minPower - pidfController.getComputedOutput(motorArm2.getCurrentPosition());


        while(opModeIsActive()){
            telemetry.addData("encoder position: ", motorArm.getCurrentPosition());
            telemetry.addData("power",
                    pidfController.getComputedOutput(motorArm.getCurrentPosition()));
            telemetry.update();

            if(motorArm.getCurrentPosition() < encoderDegreesToAttain){
                motorArm.setPower(powerM);
                motorArm2.setPower(powerM2);
            } else {
                motorArm.setPower(powerS);
                motorArm2.setPower(powerS2);
            }
        }
        motorArm.setPower(0);
    }
}
