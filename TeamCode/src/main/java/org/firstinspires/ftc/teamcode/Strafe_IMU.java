package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Strafe_IMU {
    public static double COUNTS_PER_MOTOR_REV = 28;
    public static double DRIVE_GEAR_REDUCTION = 20;
    public static double WHEEL_CIRCUMFERENCE_CM = 7.5 * Math.PI;
    public static double COUNTS_PER_WHEEL_REV = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    public static double COUNTS_PER_CM = COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_CM;

    public static double CORRECTION_GAIN = 0;

    public void moveStrafe(DcMotorEx leftF, DcMotorEx leftB, DcMotorEx rightF, DcMotorEx rightB, IMU imu, double distanceCM, double speed) {

        int targetTicks = (int) (distanceCM * COUNTS_PER_CM);
        int startPosition = leftF.getCurrentPosition();
        double initialAngle = getHeading(imu);

        leftF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (Math.abs(leftF.getCurrentPosition() - startPosition) < targetTicks) {
            double currentAngle = getHeading(imu);
            double error = currentAngle - initialAngle; // Diferença entre o ângulo atual e o inicial
            double correction = error * CORRECTION_GAIN;

            // Ajusta a velocidade dos motores para corrigir a direção
            leftF.setVelocity(speed - correction);
            leftB.setVelocity(-(speed - correction));
            rightF.setVelocity(-(speed + correction));
            rightB.setVelocity(speed + correction);
        }

        // Para os motores ao finalizar
        leftF.setVelocity(0);
        leftB.setVelocity(0);
        rightF.setVelocity(0);
        rightB.setVelocity(0);
    }

    // Método utilitário para obter o ângulo de giro (Yaw)
    private static double getHeading(IMU imu) {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.DEGREES);
    }
}
