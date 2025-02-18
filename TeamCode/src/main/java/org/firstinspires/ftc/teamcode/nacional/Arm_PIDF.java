package org.firstinspires.ftc.teamcode.nacional;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class Arm_PIDF extends LinearOpMode {

    DcMotor motorArm;

    @Override
    public void runOpMode() throws InterruptedException{

        int encoderDegreesToAttain = 200;
        double minPower = 0.01;
        double maxPower = 0.5;
        PIDFController pidfController = new PIDFController(1, 0, 0, 1);
        pidfController.setInputRange(50, 4000);
        pidfController.setSetPoint(encoderDegreesToAttain);
        pidfController.setOutputRange(minPower, maxPower);

        motorArm = hardwareMap.get(DcMotor.class, "armMotor1");

        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("encoder position: ", motorArm.getCurrentPosition());
            telemetry.addData("power",
                    pidfController.getComputedOutput(motorArm.getCurrentPosition()));
            telemetry.update();

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
