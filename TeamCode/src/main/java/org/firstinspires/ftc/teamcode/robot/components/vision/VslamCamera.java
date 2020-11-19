package org.firstinspires.ftc.teamcode.robot.components.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

public class VslamCamera implements Localizer {
    //Remember all measurements for ftclib geometry are in meters

    //How far camera is in front from the center of the robot
    public static final double T265_OFFSET_FRONT = -(MecanumDriveTrain.DRIVE_TRAIN_LENGTH/2 + 0.5*Field.MM_PER_INCH)/1000;
    //How far camera is to the left of center
    public static final double T265_CAMERA_OFFSET_LEFT = 0;
    public static final double T265_ROTATION = 180;

    private static T265Camera t265Camera = null;
    private static com.acmerobotics.roadrunner.geometry.Pose2d lastPose = new com.acmerobotics.roadrunner.geometry.Pose2d();
    private static com.acmerobotics.roadrunner.geometry.Pose2d pose2dVelocity = new com.acmerobotics.roadrunner.geometry.Pose2d();
    private static double lastUpdateTime = 0;
    private static NanoClock clock = NanoClock.system();
    private Transform2d t265LocationOnRobot =
            new Transform2d(new Translation2d(T265_OFFSET_FRONT, T265_CAMERA_OFFSET_LEFT),
                    new Rotation2d(Math.toRadians(T265_ROTATION)));
    private static final Object synchronizationObject = new Object();
    private volatile boolean isInitialized = false;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    public VslamCamera(HardwareMap hardwareMap) {
            Thread vslamInitializationThread = new Thread(new Runnable() {
                @Override
                public void run() {
                    synchronized (synchronizationObject) {
                        if (t265Camera == null) {
                            t265Camera = new T265Camera(t265LocationOnRobot, 0.8, hardwareMap.appContext);
                            try {//start our camera
                                RobotLog.i("Starting camera");
                                t265Camera.start();
                                RobotLog.i("Started camera");
                            }
                            catch (Throwable e) {
                                RobotLog.i("Exception starting camera: " + e);
                            }
                        }
                        t265Camera.setPose(new Pose2d(0, 0, new Rotation2d(0)));
                        synchronized (synchronizationObject) {
                            isInitialized = true;
                        }
                    }
                }
            });
            vslamInitializationThread.start();
    }

    /**
     * Set robot starting position based on alliance selection and left or right starting position
     * @param alliance
     * @param startingPosition
     */
    public void setStartingPose(Alliance.Color alliance, Field.StartingPosition startingPosition) {
        Pose2d startingPose;
        double robotDriveTrainCenterStart = -(Field.FIELD_WIDTH/2 - MecanumDriveTrain.DRIVE_TRAIN_LENGTH /2 - 3*Field.MM_PER_INCH)/1000;
        //calculate the starting pose of the robot
        if (alliance == Alliance.Color.RED) { //red alliance
            if (startingPosition == Field.StartingPosition.RIGHT) {
                startingPose = new Pose2d(
                        robotDriveTrainCenterStart,
                        -(Field.TILE_WIDTH*2 + Field.TAPE_WIDTH - Robot.WIDTH/2)/1000,
                        new Rotation2d(Math.toRadians(180)));
            }
            else {
                startingPose = new Pose2d(
                        robotDriveTrainCenterStart,
                        -(Field.FIELD_WIDTH + Robot.WIDTH/2)/1000,
                        new Rotation2d(Math.toRadians(180)));
            }
        }
        else { //blue alliance
            if (startingPosition == Field.StartingPosition.LEFT) {
                startingPose = new Pose2d(
                        robotDriveTrainCenterStart,
                        (Field.TILE_WIDTH*2 + Field.TAPE_WIDTH - Robot.WIDTH/2)/1000,
                        new Rotation2d(Math.toRadians(180)));
            }
            else {
                startingPose = new Pose2d(
                        robotDriveTrainCenterStart,
                        (Field.FIELD_WIDTH + Robot.WIDTH/2)/1000,
                        new Rotation2d(Math.toRadians(180)));
            }
        }
        //set the pose of the camera
        t265Camera.setPose(startingPose);
    }

    public boolean isInitialized() {
        synchronized (synchronizationObject) {
            return this.isInitialized;
        }
    }

    public Pose2d getPosition() {
        T265Camera.CameraUpdate cameraUpdate = t265Camera.getLastReceivedCameraUpdate();
        if (cameraUpdate != null) {
            return cameraUpdate.pose;
        }
        else {
            return null;
        }
    }

    //Methods to implement roadrunner localizer
    @NotNull
    @Override
    public com.acmerobotics.roadrunner.geometry.Pose2d getPoseEstimate() {
        return lastPose;
    }

    @Override
    public void setPoseEstimate(@NotNull com.acmerobotics.roadrunner.geometry.Pose2d pose2d) {
        /*
        Pose2d newPose = new Pose2d(
                -pose2d.getX() / Field.M_PER_INCH,
                -pose2d.getY() / Field.M_PER_INCH,
                new Rotation2d(pose2d.getHeading()));
        t265Camera.setPose(newPose);

         */
    }

    @Nullable
    @Override
    public com.acmerobotics.roadrunner.geometry.Pose2d getPoseVelocity() {
        return pose2dVelocity;
    }

    @Override
    public void update() {
        //find time elapsed
        double currentTime = clock.seconds();
        double timeElapsed = currentTime - lastUpdateTime;
        lastUpdateTime = currentTime;
        Pose2d currentPosition = this.getPosition();
        try {
            //find current position in the coordinates and units of measure that roadrunner understands
            double currentX = currentPosition.getX() / Field.M_PER_INCH;
            double currentY = currentPosition.getY() / Field.M_PER_INCH;
            double currentTheta = currentPosition.getHeading();

            pose2dVelocity = new com.acmerobotics.roadrunner.geometry.Pose2d(
                    (currentX - lastPose.getX()) / timeElapsed,
                    (currentY - lastPose.getY()) / timeElapsed,
                    (currentTheta - lastPose.getHeading()) / timeElapsed);
            //set last pose to be the current one
            lastPose = new com.acmerobotics.roadrunner.geometry.Pose2d(
                    currentX,
                    currentY,
                    currentTheta);
        }
        catch (Throwable e) {
            RobotLog.ee("Vslam", e, "Error getting position");
        }
        //RobotLog.i(currentPosition.toString() + ", " + lastPose.toString());
    }
}
