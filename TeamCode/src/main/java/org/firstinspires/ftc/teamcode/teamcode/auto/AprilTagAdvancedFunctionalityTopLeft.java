package org.firstinspires.ftc.teamcode.auto;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Autonomous
public class AprilTagAdvancedFunctionalityTopLeft extends LinearOpMode {

    public void runOpMode() throws InterruptedException {

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d topLeftStartPose = new Pose2d(-63, 10, Math.toRadians(0));

        drive.setPoseEstimate(topLeftStartPose);

        Trajectory startTopLeftTraj = drive.trajectoryBuilder(topLeftStartPose)
                .splineToConstantHeading(new Vector2d(-43, 22), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(-37, 22),
                        SampleMecanumDrive.getVelocityConstraint(8, 49, 14.65),
                        SampleMecanumDrive.getAccelerationConstraint(49)
                )
                .addSpatialMarker(new Vector2d(-37, 22), () -> {
                    //drop the pixel
                })
                .back((7.5),
                        SampleMecanumDrive.getVelocityConstraint(8, 49, 14.65),
                        SampleMecanumDrive.getAccelerationConstraint(49)
                )
                .lineToLinearHeading(new Pose2d(-41.25, 22, Math.toRadians(90))) //back up and face the left AprilTag
                .build();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

            //ADD OBJECT DETECTION CODE HERE. DETERMINE IF NEED TO GO TO THE LEFT, MIDDLE, RIGHT
            //ASSIGNING A STRING VALUE WITH THE TRAJ WILL LET THE CAMERA KNOW WHICH APRILTAG IT HAS TO SENSE
            //E.G. IF LEFT, THEN USE CONDITIONAL STATEMENTS TO ENSURE THAT THE CAMERA FOLLOWS THE APRILTAG WITH ID OF 1

            // New trajectory
            // int LEFT TRAJECTORY = 1
            // int MIDDLE TRAJECTORY = 2
            // int RIGHT TRAJECTORY = 3
            int leftTrajectory = 1;
            int middleTrajectory = 2;
            int rightTrajectory = 3;
            int Trajectory = 3;

            AprilTagDetection tag = null;
            if (tagProcessor.getDetections().size() > 0) {
                tag = tagProcessor.getDetections().get(0);
            }

            double x = tag.ftcPose.x;
            double y = tag.ftcPose.y;

            if (Trajectory == leftTrajectory) {
                drive.followTrajectoryAsync(startTopLeftTraj);

                //adjustedAngle = .getBearing()
                //Alternatively use trig to calculate the angle bearing using the x, y values:
                double adjustedAngle = Math.round(Math.atan(x/y));
                Trajectory topLeftTraj_toAprilTag = drive.trajectoryBuilder(startTopLeftTraj.end())
                    //Using headings this way will ensure that the camera sees the left aprilTag first
                    .lineToConstantHeading(new Vector2d(-41.25, 50.5),
                                            SampleMecanumDrive.getVelocityConstraint(8, 49, 14.65),
                                            SampleMecanumDrive.getAccelerationConstraint(49)
                    )
                    .build();
                drive.followTrajectory(topLeftTraj_toAprilTag);

                //Line up the robot to the adjusted angle
                drive.turn(adjustedAngle);

                //move forward to the backdrop, slower velocity
                Trajectory topLeftTraj_toOutput_toPark = drive.trajectoryBuilder(topLeftTraj_toAprilTag.end())
                        .lineToConstantHeading(new Vector2d(-41.25, 50.5),
                                SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .addDisplacementMarker(() -> {
                            //drop into the backdrop
                        })
                        .lineToConstantHeading(new Vector2d(-41.25, 48))
                        .lineToLinearHeading(new Pose2d(-58.5, 48, Math.toRadians(0)))
                        .lineToLinearHeading(new Pose2d(-58.5, 65, Math.toRadians(270)))
                        .build();
                drive.followTrajectory(topLeftTraj_toOutput_toPark);

            } else if (Trajectory == middleTrajectory) {
                
            } else { //right trajectory

            }
        }
    }
}
