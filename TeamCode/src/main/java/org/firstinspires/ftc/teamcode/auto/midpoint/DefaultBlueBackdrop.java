package org.firstinspires.ftc.teamcode.auto.midpoint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class DefaultBlueBackdrop extends LinearOpMode {

    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d StartPose = new Pose2d(-63, 10, Math.toRadians(0));

        drive.setPoseEstimate(StartPose);

        TrajectorySequence defaultTrajectory = drive.trajectorySequenceBuilder(StartPose)
                .lineToLinearHeading(new Pose2d(-35, 29, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(-35, 42.5))
                .lineToConstantHeading(new Vector2d(-35, 50.5),
                        SampleMecanumDrive.getVelocityConstraint(8, 49, 14.65),
                        SampleMecanumDrive.getAccelerationConstraint(49)
                )
                .addSpatialMarker(new Vector2d(-35, 50.5), () -> {
                    //Drop into the backdrop
                })
                .waitSeconds(2) //simulate dropping
                .lineToConstantHeading(new Vector2d(-41.25, 48))
                .lineToLinearHeading(new Pose2d(-58.5, 48, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-58.5, 65, Math.toRadians(270)))
                .build();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            drive.followTrajectorySequenceAsync(defaultTrajectory);
        }
    }
}
