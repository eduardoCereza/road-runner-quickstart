package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Gyro{
    public void turnToAngle(DcMotorEx leftF, DcMotorEx leftB, DcMotorEx rightF, DcMotorEx rightB, IMU imu, double targetAngle, double turnSpeed) {
        // Pega o ângulo inicial
        double initialAngle = getHeading(imu);
        double currentAngle;

        leftF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Gira até atingir o ângulo desejado
        do {
            currentAngle = getHeading(imu);

            if (targetAngle > 0) {
                // Gira para a direita
                leftF.setVelocity(-turnSpeed);
                rightB.setVelocity(turnSpeed);
                leftB.setVelocity(-turnSpeed);
                rightF.setVelocity(turnSpeed);
            } else {
                // Gira para a esquerda
                leftF.setVelocity(turnSpeed);
                rightB.setVelocity(-turnSpeed);
                leftB.setVelocity(turnSpeed);
                rightF.setVelocity(-turnSpeed);
            }

        } while (Math.abs(currentAngle - initialAngle) < Math.abs(targetAngle));

        // Para os motores ao finalizar
        leftF.setVelocity(0);
        rightB.setVelocity(0);
        leftB.setVelocity(0);
        rightF.setVelocity(0);
    }

    // Método utilitário para obter o ângulo de giro (Yaw)
    private static double getHeading(IMU imu) {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.DEGREES);
    }
}
