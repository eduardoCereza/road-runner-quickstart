package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "First Road Runner", group = "Autonomous")
public class FirstRoadRunnerAuto extends LinearOpMode {

    @Override
    public void runOpMode(){

        Pose2d beginPose = new Pose2d(0, 0, 0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                drive.actionBuilder(beginPose)

                        .splineTo(new Vector2d(-0.23, -33.04), Math.toRadians(90))
                        .build()
        );
    }
}
