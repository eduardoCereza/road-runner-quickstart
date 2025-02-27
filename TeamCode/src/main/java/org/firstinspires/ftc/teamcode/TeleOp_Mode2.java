package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Auto RoadRunner", group = "Autonomous")
public class TeleOp_Mode2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Inicializa o drive baseado no Road Runner
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        // Define a posição inicial do robô (0, 0) virado para a frente
        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        // Criar um trajeto para ir até (30, 30) e girar 90 graus
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(30, 30, Math.toRadians(90)))
                .build();

        // Criar um trajeto para voltar ao ponto inicial
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(180)))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        // Executa os trajetos
        drive.followTrajectory(traj1);
        sleep(500); // Espera 0,5s para estabilizar
        drive.followTrajectory(traj2);
    }
}
