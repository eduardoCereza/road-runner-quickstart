package org.firstinspires.ftc.teamcode.nacional;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class Arm_Move {
    public void move_Arm(DcMotor motorArm, int encoderDegreesToAttain){

        double minPower = 0.01;
        double maxPower = 0.5;
        PIDFController pidfController = new PIDFController(1, 0, 0, 1);
        pidfController.setInputRange(50, 4000);
        pidfController.setSetPoint(encoderDegreesToAttain);
        pidfController.setOutputRange(minPower, maxPower);

        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(motorArm.isBusy()){
            if(motorArm.getCurrentPosition() < encoderDegreesToAttain){
                motorArm.setPower(minPower +
                        pidfController.getComputedOutput(motorArm.getCurrentPosition()));
            } else {
                motorArm.setPower(minPower -
                        pidfController.getComputedOutput(motorArm.getCurrentPosition()));
            }
        }
        motorArm.setPower(0);
    }
}
