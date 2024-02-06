package org.firstinspires.ftc.teamcode.auto.advanced;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous
public class RedBackdrop extends LinearOpMode {

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

        Pose2d StartPose = new Pose2d(63, 12, Math.toRadians(0));

        drive.setPoseEstimate(StartPose);

        TrajectorySequence defaultTrajectory = drive.trajectorySequenceBuilder(StartPose)
                .lineToLinearHeading(new Pose2d(35, 29, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(35, 42.5))
                .lineToConstantHeading(new Vector2d(35, 50.5),
                        SampleMecanumDrive.getVelocityConstraint(8, 49, 14.65),
                        SampleMecanumDrive.getAccelerationConstraint(49)
                )
                .addSpatialMarker(new Vector2d(35, 50.5), () -> {
                    //Drop into the backdrop
                })
                .waitSeconds(2) //simulate dropping
                .lineToConstantHeading(new Vector2d(41.25, 48))
                .lineToLinearHeading(new Pose2d(58.5, 48, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(58.5, 65, Math.toRadians(270)))
                .build();

        Trajectory LeftTrajectory = drive.trajectoryBuilder(StartPose)
                .lineToConstantHeading(new Vector2d(40, 12))
                .lineToConstantHeading(new Vector2d(29, 12),
                        SampleMecanumDrive.getVelocityConstraint(8, 49, 14.65),
                        SampleMecanumDrive.getAccelerationConstraint(49)
                )
                .lineToLinearHeading(new Pose2d(29, 13, Math.toRadians(270)))
                .lineToConstantHeading(new Vector2d(29, 9.5),
                        SampleMecanumDrive.getVelocityConstraint(8, 49, 14.65),
                        SampleMecanumDrive.getAccelerationConstraint(49)
                )
                .addSpatialMarker(new Vector2d(29, 9.5), () -> {
                    // Drop the pixel precisely at (29, 9.5)
                })
                .back((5),
                        SampleMecanumDrive.getVelocityConstraint(8, 49, 14.65),
                        SampleMecanumDrive.getAccelerationConstraint(49)
                )
                .lineToLinearHeading(new Pose2d(32, 25, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(41, 42.5, Math.toRadians(90))) //back up and face the left AprilTag
                .build();

        Trajectory MiddleTrajectory = drive.trajectoryBuilder(StartPose)
                .lineToConstantHeading(new Vector2d(40, 12))
                .lineToConstantHeading(new Vector2d(33, 12),
                        SampleMecanumDrive.getVelocityConstraint(8, 49, 14.65),
                        SampleMecanumDrive.getAccelerationConstraint(49)
                )
                .addSpatialMarker(new Vector2d(33, 12), () -> {
                    //Drop the pixel on the line
                })
                .forward((6.5),
                        SampleMecanumDrive.getVelocityConstraint(8, 49, 14.65),
                        SampleMecanumDrive.getAccelerationConstraint(49)
                )
                .lineToLinearHeading(new Pose2d(35, 42.5, Math.toRadians(90)))
                .build();

        Trajectory RightTrajectory = drive.trajectoryBuilder(StartPose)
                .splineToConstantHeading(new Vector2d(43, 22), Math.toRadians(0)) //change to a lower value
                .lineToConstantHeading(new Vector2d(39, 22),
                        SampleMecanumDrive.getVelocityConstraint(8, 49, 14.65),
                        SampleMecanumDrive.getAccelerationConstraint(49)
                )
                .addSpatialMarker(new Vector2d(39, 22), () -> {
                    //drop the pixel
                })
                .forward((12),
                        SampleMecanumDrive.getVelocityConstraint(8, 49, 14.65),
                        SampleMecanumDrive.getAccelerationConstraint(49)
                )
                .lineToLinearHeading(new Pose2d(45, 22, Math.toRadians(90))) //back up and face the left AprilTag
                .lineToConstantHeading(new Vector2d(41.25, 42.5))
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
            int Trajectory = 0;

            AprilTagDetection tag = null;
            if (tagProcessor.getDetections().size() > 0) {
                tag = tagProcessor.getDetections().get(0);
            }

            double x = tag.ftcPose.x;
            double y = tag.ftcPose.y;

            if (Trajectory == leftTrajectory) {
                drive.followTrajectoryAsync(LeftTrajectory);

                //adjustedAngle = .getBearing()
                //Alternatively use trig to calculate the angle bearing using the x, y values:
                double adjustedAngle = Math.round(Math.atan(x/y));
                Trajectory leftAprilTag = drive.trajectoryBuilder(LeftTrajectory.end())
                        //Using headings this way will ensure that the camera sees the left aprilTag first
                        .lineToConstantHeading(new Vector2d(41, 50),
                                SampleMecanumDrive.getVelocityConstraint(8, 49, 14.65),
                                SampleMecanumDrive.getAccelerationConstraint(49)
                        )
                        .build();
                drive.followTrajectory(leftAprilTag);

                //Line up the robot to the adjusted angle
                drive.turn(adjustedAngle);

                //move forward to the backdrop, slower velocity
                Trajectory leftOutputPark = drive.trajectoryBuilder(leftAprilTag.end())
                        .addDisplacementMarker(() -> {
                            //drop into the backdrop
                        })
                        //Park
                        .back(5)
                        .lineToConstantHeading(new Vector2d(41, 43))
                        .lineToLinearHeading(new Pose2d(58.5, 43, Math.toRadians(180)))
                        .lineToLinearHeading(new Pose2d(58.5, 65, Math.toRadians(270)))
                        .build();
                drive.followTrajectory(leftOutputPark);

            } else if (Trajectory == middleTrajectory) {
                drive.followTrajectoryAsync(MiddleTrajectory);

                double adjustedAngle = Math.round(Math.atan(x/y));
                Trajectory middleAprilTag = drive.trajectoryBuilder(MiddleTrajectory.end())
                        //Using headings this way will ensure that the camera sees the left aprilTag first
                        .lineToConstantHeading(new Vector2d(35, 50.5),
                                SampleMecanumDrive.getVelocityConstraint(8, 49, 14.65),
                                SampleMecanumDrive.getAccelerationConstraint(49)
                        )
                        .build();
                drive.followTrajectory(middleAprilTag);

                //Line up the robot to the adjusted angle
                drive.turn(adjustedAngle);

                //move forward to the backdrop, slower velocity
                Trajectory middleOutputPark = drive.trajectoryBuilder(middleAprilTag.end())
                        .addDisplacementMarker(() -> {
                            //Drop into backdrop
                        })
                        //Park
                        .lineToConstantHeading(new Vector2d(41.25, 48))
                        .lineToLinearHeading(new Pose2d(58.65, 48, Math.toRadians(180)))
                        .lineToLinearHeading(new Pose2d(58.65, 65, Math.toRadians(270)))
                        .build();
                drive.followTrajectory(middleOutputPark);

            } else if (Trajectory == rightTrajectory) { //right trajectory
                drive.followTrajectoryAsync(RightTrajectory);

                double adjustedAngle = Math.round(Math.atan(x/y));
                Trajectory rightAprilTag = drive.trajectoryBuilder(RightTrajectory.end())
                        //Using headings this way will ensure that the camera sees the left aprilTag first
                        .lineToConstantHeading(new Vector2d(41.25, 50.5),
                                SampleMecanumDrive.getVelocityConstraint(8, 49, 14.65),
                                SampleMecanumDrive.getAccelerationConstraint(49)
                        )
                        .build();
                drive.followTrajectory(rightAprilTag);

                //Line up the robot to the adjusted angle
                drive.turn(adjustedAngle);

                //move forward to the backdrop, slower velocity
                Trajectory rightOutputPark = drive.trajectoryBuilder(rightAprilTag.end())
                        .addDisplacementMarker(() -> {
                            //drop into the backdrop
                        })
                        //Park
                        .lineToConstantHeading(new Vector2d(41.25, 48))
                        .lineToLinearHeading(new Pose2d(58.5, 48, Math.toRadians(180)))
                        .lineToLinearHeading(new Pose2d(58.5, 65, Math.toRadians(270)))
                        .build();
                drive.followTrajectory(rightOutputPark);

            } else { //DEFAULT
                drive.followTrajectorySequenceAsync(defaultTrajectory);
            }
        }
    }
}

