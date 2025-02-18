package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Arm_PID_Class {
    public void armPID (DcMotorEx left, DcMotorEx right, int encoderDegreesToAttain) {
        double minPower = 0.01;
        double maxPower = 0.5;
        PController pController = new PController(0.03);
        pController.setInputRange(50, 500);
        pController.setSetPoint(encoderDegreesToAttain);
        pController.setOutputRange(minPower, maxPower);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        while(left.isBusy()){
            if (left.getCurrentPosition() < encoderDegreesToAttain){
                left.setPower(minPower + pController.getComputedOutput(left.getCurrentPosition()));
            }
            else {
                left.setPower(minPower - pController.getComputedOutput(left.getCurrentPosition()));
            }
            left.setPower(0);
        }
    }
}
