package org.firstinspires.ftc.teamcode.robot.components.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;

public class VslamCamera {
    //Remember all measurements for ftclib geometry are in meters

    //How far camera is in front from the center of the robot
    public static final double T265_OFFSET_FRONT = -Robot.LENGTH/1000/2;
    //How far camera is to the left of center
    public static final double T265_CAMERA_OFFSET_LEFT = 0;
    public static final double T265_ROTATION = 180;

    private static T265Camera t265Camera = null;
    private Transform2d t265LocationOnRobot =
            new Transform2d(new Translation2d(T265_OFFSET_FRONT, T265_CAMERA_OFFSET_LEFT),
                    new Rotation2d(Math.toRadians(T265_ROTATION)));
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public void init(HardwareMap hardwareMap, Alliance.Color alliance, Field.StartingPosition startingPosition) {
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
        if (t265Camera == null) {
            t265Camera = new T265Camera(t265LocationOnRobot, 0.8, hardwareMap.appContext);
        }
        //set starting location of our robot by letting the camera know
        t265Camera.setPose(startingPose);
        try {
            //start our camera
            t265Camera.start();
        }
        catch (Throwable e) {}
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
}
