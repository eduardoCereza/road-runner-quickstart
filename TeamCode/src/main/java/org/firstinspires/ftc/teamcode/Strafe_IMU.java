package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Strafe_IMU {
    public void move(double distance, DcMotorEx leftF, DcMotorEx leftB, DcMotorEx rightF, DcMotorEx rightB, IMU imu) {
        double COUNTS_PER_MOTOR_REV = 28;
        double DRIVE_GEAR_REDUCTION = 20;
        double WHEEL_CIRCUMFERENCE_CM = 7.5 * Math.PI;

        double COUNTS_PER_WHEEL_REV = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
        double COUNTS_PER_CM = COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_CM;

        double kP_IMU = 0.01; // Fator proporcional para correção com o IMU
        double MAX_POWER = 0.6;
        double MIN_POWER = 0.2;

        leftF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        int targetPosition = (int) (distance * COUNTS_PER_CM);

        // Define as posições alvo para o movimento lateral
        leftF.setTargetPosition(targetPosition);
        rightF.setTargetPosition(-targetPosition);
        leftB.setTargetPosition(-targetPosition);
        rightB.setTargetPosition(targetPosition);

        leftF.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightF.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftB.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightB.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Obtém o ângulo inicial do IMU
        double initialAngle = getHeading(imu);

        while (leftF.isBusy() && rightF.isBusy() && leftB.isBusy() && rightB.isBusy()) {
            // Lê o ângulo atual do IMU
            double currentAngle = getHeading(imu);
            double angleError = currentAngle - initialAngle;

            // Calcula correção para manter o robô alinhado
            double correction = kP_IMU * angleError;

            // Ajusta potência dos motores
            double leftPower = MAX_POWER - correction;
            double rightPower = MAX_POWER + correction;

            // Limita a potência dentro dos valores permitidos
            leftPower = Math.max(MIN_POWER, Math.min(MAX_POWER, leftPower));
            rightPower = Math.max(MIN_POWER, Math.min(MAX_POWER, rightPower));

            leftF.setPower(leftPower);
            leftB.setPower(-leftPower);
            rightF.setPower(rightPower);
            rightB.setPower(-rightPower);
        }

        leftF.setPower(0);
        leftB.setPower(0);
        rightB.setPower(0);
        rightF.setPower(0);
    }

    // Método para obter o ângulo atual (yaw) do IMU BHI260AP
    private static double getHeading(IMU imu) {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.DEGREES);
    }
}
