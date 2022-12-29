package org.firstinspires.ftc.teamcode.opmodes;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.SleeveDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.SleeveDetector;

@Autonomous
public class CycleAutoRedRightLowV2 extends LinearOpMode {
    Pose2d START_POSE = new Pose2d(33, -64, Math.toRadians(0));
    Robot robot;
    SleeveDetector detector = new SleeveDetector();
    SleeveDetectionPipeline.Color parkingPos = SleeveDetectionPipeline.Color.BLUE;
    private ElapsedTime timer;

    public double liftHigh = 1000;
    public double liftMid = 500;
    public double liftLow = 200;
    public double liftGround = 0;
    public double liftIdle = 200;
    public double liftIntaking = 0;

    public double turretFront = 0;
    public double turretLeft = 330;
    public double turretBack = 660;
    public double turretRight = 990;

    //change after tuning horizontal slides
    public double hzslidesout = 0.6;
    public double hzslidesin = 0.3;

    public void runOpMode() {
        robot = new Robot(telemetry, hardwareMap);
        timer = new ElapsedTime();
        robot.init();
        robot.intake.closeClaw();
        sleep(1500);
        robot.intake.dropArm();
        detector.init(hardwareMap, telemetry);
//        robot.lift.setOpmode("auto");

        TrajectorySequence parking1 = robot.drive.trajectorySequenceBuilder(START_POSE)
                .setVelConstraint(robot.drive.getVelocityConstraint(30, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .setReversed(true)
                // Preplaced
                //.strafeLeft(60)
                .lineToLinearHeading(new Pose2d(36,-12,Math.toRadians(0)))
//                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
//                    robot.lift.setTargetHeight(liftHigh);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//                    robot.turret.setTargetAngle(turretRight/2);
//                    robot.intake.setArmPos(0.55);
//                })
//                .waitSeconds(0.2)
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                    robot.lift.setHorizontalPosition(hzslidesout);
//                    robot.intake.openClaw();
//                })
//                .waitSeconds(0.5)
//                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
//                    robot.turret.setTargetAngle(turretBack);
//                    robot.lift.setHorizontalPosition(hzslidesin);
//                })

                // Cycle #1

//                .addTemporalMarker(() -> {
//                })
//                .setReversed(false)
//                .waitSeconds(0.9)
//                .setVelConstraint(robot.drive.getVelocityConstraint(20, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
//                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
//                    robot.lift.setTargetHeight(7.5);
//                    robot.intake.centerArm();
//                })
//                .waitSeconds(0.2)
//                .lineToLinearHeading(new Pose2d(52, -12, Math.toRadians(180)))
//                //pick up cone
//                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
//                    robot.lift.setHorizontalPosition(hzslidesout);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
//                    robot.intake.closeClaw();
//                })
//                .waitSeconds(0.5)
//                .addTemporalMarker(() -> {
//                    robot.lift.setTargetHeight(liftLow);
//                    robot.lift.setHorizontalPosition(hzslidesin);
//                    robot.turret.setTargetAngle(575);
//                })
//                .waitSeconds(0.7)
//                .addTemporalMarker(() -> {
//                    robot.lift.setHorizontalPosition(hzslidesout);
//                })
//                .waitSeconds(1)
//                .addTemporalMarker(() -> {
//                    robot.lift.setTargetHeight(liftLow);
//                    robot.intake.fullyOpenClaw();
//                })
//                .waitSeconds(0.3)
//                .addTemporalMarker(() -> {
//                    robot.turret.setTargetAngle(turretBack);
//                })
//                .waitSeconds(0.3)
                .build();


        robot.drive.setPoseEstimate(START_POSE);

        // Waiting for start
        while (!isStarted() && !isStopRequested()) {
            parkingPos = detector.getColor();
            telemetry.addData("timer", timer.milliseconds());
            telemetry.update();
        }

        // Start...
        detector.stop();
        waitForStart();

        while (opModeIsActive()) {
            Drivetrain drive = new Drivetrain(hardwareMap);
            drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("turret pos", robot.turret.getCurrentAngle());
            telemetry.addData("turret target", robot.turret.getTargetAngle());
//            telemetry.addData("opmode", robot.lift.getOpmode());
            telemetry.addData("slide pos", robot.lift.getCurrentHeight());
            telemetry.addData("Robot X:", poseEstimate.getX());
            telemetry.addData("Robot Y:", poseEstimate.getY());
            telemetry.addData("Robot Heading:", poseEstimate.getHeading());
            telemetry.update();
            robot.update();
        }
    }
}