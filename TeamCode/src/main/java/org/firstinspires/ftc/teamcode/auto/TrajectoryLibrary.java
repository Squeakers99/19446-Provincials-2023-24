package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class TrajectoryLibrary extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d topLeftStartPose = new Pose2d(-70, 10, Math.toRadians(0));

        //Top Left Start Trajectory, needs camera detection
        Trajectory topLeftstartTraj = drive.trajectoryBuilder(topLeftStartPose)
                .splineToConstantHeading(new Vector2d(-37, 22), Math.toRadians(0)) //change to a lower value
                .addDisplacementMarker(() -> {
                    //drop the pixel
                })
                .lineToLinearHeading(new Pose2d(-41.25, 22, Math.toRadians(90))) //back up and face the left AprilTag

                //Using headings this way will ensure that the camera sees the left aprilTag first
                .lineToConstantHeading(new Vector2d(-41.25, 50))
                .addDisplacementMarker(() -> {
                    //drop into the backdrop
                })
                .build();

        //Bottom left start trajectory
    }
}
