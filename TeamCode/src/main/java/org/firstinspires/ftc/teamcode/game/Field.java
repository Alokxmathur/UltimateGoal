package org.firstinspires.ftc.teamcode.game;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.roadrunner.drive.PhoebeRoadRunnerDrive;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;

/**
 * Created by Silver Titans on 9/16/17.
 */

public class Field {
    public static final float MM_PER_INCH        = 25.4f;
    public static final double M_PER_INCH = MM_PER_INCH/1000;
    //the width of each tile
    public static final double TILE_WIDTH = 24 * MM_PER_INCH;
    // the width of the FTC field (from the center point to the outer panels)
    public static final double FIELD_WIDTH = 6*TILE_WIDTH;
    public static final double TAPE_WIDTH = 2*MM_PER_INCH;

    public static volatile boolean initialized = false;
    public static Object mutex = new Object();

    public static boolean isInitialized() {
        synchronized (mutex) {
            return initialized;
        }
    }

    public enum RingCount {
        ONE, NONE, FOUR
    }

    public enum StartingPosition {
        LEFT, RIGHT
    }
    public static final double ROBOT_STARTING_X = -(Field.FIELD_WIDTH/2 - MecanumDriveTrain.DRIVE_TRAIN_LENGTH/2 - 3*Field.MM_PER_INCH);
    public static final Pose2d RED_RIGHT_STARTING_POSE = new Pose2d(
            ROBOT_STARTING_X/MM_PER_INCH,
            -(Field.TILE_WIDTH*2 + Field.TAPE_WIDTH - Robot.WIDTH/2)/MM_PER_INCH);
    public static final Pose2d RED_LEFT_STARTING_POSE = new Pose2d(
            ROBOT_STARTING_X/MM_PER_INCH,
            -(Field.TILE_WIDTH + Robot.WIDTH/2)/MM_PER_INCH);
    public static final Pose2d BLUE_RIGHT_STARTING_POSE = new Pose2d(
            ROBOT_STARTING_X/MM_PER_INCH,
            (Field.FIELD_WIDTH + Robot.WIDTH/2)/MM_PER_INCH);
    public static final Pose2d BLUE_LEFT_STARTING_POSE = new Pose2d(
            ROBOT_STARTING_X/MM_PER_INCH,
            (Field.TILE_WIDTH*2 + Field.TAPE_WIDTH - Robot.WIDTH/2)/MM_PER_INCH);

    public static final Vector2d RED_NONE_DEPOSIT_VECTOR = new Vector2d(6, -68);
    public static final Vector2d RED_ONE_DEPOSIT_VECTOR = new Vector2d(30, -50);
    public static final Vector2d RED_FOUR_DEPOSIT_VECTOR = new Vector2d(54, -68);
    public static final Vector2d BLUE_NONE_DEPOSIT_VECTOR = new Vector2d(6, 68);
    public static final Vector2d BLUE_ONE_DEPOSIT_VECTOR = new Vector2d(30, 50);
    public static final Vector2d BLUE_FOUR_DEPOSIT_VECTOR = new Vector2d(54, 68);

    public static final Pose2d RED_SECOND_WOBBLE_PICK_UP_LEFT = new Pose2d(
            -47,
            -(Field.TILE_WIDTH + 9*Field.MM_PER_INCH + MecanumDriveTrain.DRIVE_TRAIN_LENGTH/2)/MM_PER_INCH,
            Math.toRadians(90));
    public static final Pose2d RED_SECOND_WOBBLE_PICK_UP_RIGHT = new Pose2d(
            -49,
            -(2*Field.TILE_WIDTH - 9*Field.MM_PER_INCH - MecanumDriveTrain.DRIVE_TRAIN_LENGTH/2)/MM_PER_INCH,
            Math.toRadians(90));
    public static final Pose2d BLUE_SECOND_WOBBLE_PICK_UP_LEFT = new Pose2d(
            -47,
            (2*Field.TILE_WIDTH - 9*Field.MM_PER_INCH - MecanumDriveTrain.DRIVE_TRAIN_LENGTH/2)/MM_PER_INCH,
            Math.toRadians(90));
    public static final Pose2d BLUE_SECOND_WOBBLE_PICK_UP_RIGHT = new Pose2d(
            -49,
            (Field.TILE_WIDTH + 9*Field.MM_PER_INCH + MecanumDriveTrain.DRIVE_TRAIN_LENGTH/2)/MM_PER_INCH,
            Math.toRadians(90));

    static Pose2d[][] startingPoses = new Pose2d[Alliance.Color.values().length][StartingPosition.values().length];

    static Trajectory[][][] dropFirstWobbleTrajectories = new Trajectory[Alliance.Color.values().length][StartingPosition.values().length]
            [RingCount.values().length];
    static Trajectory[][][] reachSecondWobbleTrajectories = new Trajectory[Alliance.Color.values().length][StartingPosition.values().length]
            [RingCount.values().length];
    static Trajectory[][][] dropSecondWobbleTrajectories = new Trajectory[Alliance.Color.values().length][StartingPosition.values().length]
            [RingCount.values().length];
    static Trajectory[][]navigationTrajectories = new Trajectory[Alliance.Color.values().length]
            [RingCount.values().length];

    {
        Thread initThread = new Thread(() -> {
            RobotLog.i("SilverTitans: Field initialization started");

            //initialize starting positions
            startingPoses[Alliance.Color.RED.ordinal()][StartingPosition.LEFT.ordinal()] = RED_LEFT_STARTING_POSE;
            startingPoses[Alliance.Color.RED.ordinal()][StartingPosition.RIGHT.ordinal()] = RED_RIGHT_STARTING_POSE;
            startingPoses[Alliance.Color.BLUE.ordinal()][StartingPosition.LEFT.ordinal()] = BLUE_LEFT_STARTING_POSE;
            startingPoses[Alliance.Color.BLUE.ordinal()][StartingPosition.RIGHT.ordinal()] = BLUE_RIGHT_STARTING_POSE;

            //Red-right first wobble trajectories
            dropFirstWobbleTrajectories[Alliance.Color.RED.ordinal()][StartingPosition.RIGHT.ordinal()][RingCount.NONE.ordinal()] =
                    new TrajectoryBuilder(RED_RIGHT_STARTING_POSE, PhoebeRoadRunnerDrive.getConstraints())
                            .splineToConstantHeading(new Vector2d(-48, RED_NONE_DEPOSIT_VECTOR.getY()), 0)
                            .splineTo(RED_NONE_DEPOSIT_VECTOR, 0)
                            .build();
            dropFirstWobbleTrajectories[Alliance.Color.RED.ordinal()][StartingPosition.RIGHT.ordinal()][RingCount.ONE.ordinal()] =
                    new TrajectoryBuilder(RED_LEFT_STARTING_POSE, PhoebeRoadRunnerDrive.getConstraints())
                            .splineToConstantHeading(new Vector2d(-48, RED_ONE_DEPOSIT_VECTOR.getY()), 0)
                            .splineTo(RED_ONE_DEPOSIT_VECTOR, 0)
                            .build();
            dropFirstWobbleTrajectories[Alliance.Color.RED.ordinal()][StartingPosition.RIGHT.ordinal()][RingCount.FOUR.ordinal()] =
                    new TrajectoryBuilder(RED_RIGHT_STARTING_POSE, PhoebeRoadRunnerDrive.getConstraints())
                            .splineToConstantHeading(new Vector2d(-48, RED_FOUR_DEPOSIT_VECTOR.getY()), 0)
                            .splineToConstantHeading(RED_FOUR_DEPOSIT_VECTOR, 0)
                            .build();
            //red-left first wobble trajectories
            dropFirstWobbleTrajectories[Alliance.Color.RED.ordinal()][StartingPosition.LEFT.ordinal()][RingCount.NONE.ordinal()] =
                    new TrajectoryBuilder(RED_LEFT_STARTING_POSE, PhoebeRoadRunnerDrive.getConstraints())
                            .splineToConstantHeading(new Vector2d(-24, -62), 0)
                            .splineToConstantHeading(RED_NONE_DEPOSIT_VECTOR, 0)
                            .build();
            dropFirstWobbleTrajectories[Alliance.Color.RED.ordinal()][StartingPosition.LEFT.ordinal()][RingCount.ONE.ordinal()] =
                    new TrajectoryBuilder(RED_LEFT_STARTING_POSE, PhoebeRoadRunnerDrive.getConstraints())
                            .splineToConstantHeading(new Vector2d(-24, -62), 0)
                            .splineToConstantHeading(RED_ONE_DEPOSIT_VECTOR, 0)
                            .build();
            dropFirstWobbleTrajectories[Alliance.Color.RED.ordinal()][StartingPosition.LEFT.ordinal()][RingCount.FOUR.ordinal()] =
                    new TrajectoryBuilder(RED_LEFT_STARTING_POSE, PhoebeRoadRunnerDrive.getConstraints())
                            .splineToConstantHeading(new Vector2d(-24, -62), 0)
                            .splineToConstantHeading(RED_FOUR_DEPOSIT_VECTOR, 0)
                            .build();
            //blue right first wobble trajectories
            dropFirstWobbleTrajectories[Alliance.Color.BLUE.ordinal()][StartingPosition.RIGHT.ordinal()][RingCount.NONE.ordinal()] =
                    new TrajectoryBuilder(BLUE_RIGHT_STARTING_POSE, PhoebeRoadRunnerDrive.getConstraints())
                            .splineToConstantHeading(new Vector2d(-24, 62), 0)
                            .splineToConstantHeading(BLUE_NONE_DEPOSIT_VECTOR, 0)
                            .build();
            dropFirstWobbleTrajectories[Alliance.Color.BLUE.ordinal()][StartingPosition.RIGHT.ordinal()][RingCount.ONE.ordinal()] =
                    new TrajectoryBuilder(BLUE_RIGHT_STARTING_POSE, PhoebeRoadRunnerDrive.getConstraints())
                            .splineToConstantHeading(new Vector2d(-24, 62), 0)
                            .splineToConstantHeading(BLUE_ONE_DEPOSIT_VECTOR, 0)
                            .build();
            dropFirstWobbleTrajectories[Alliance.Color.BLUE.ordinal()][StartingPosition.RIGHT.ordinal()][RingCount.FOUR.ordinal()] =
                    new TrajectoryBuilder(BLUE_RIGHT_STARTING_POSE, PhoebeRoadRunnerDrive.getConstraints())
                            .splineToConstantHeading(new Vector2d(-24, 62), 0)
                            .splineTo(BLUE_FOUR_DEPOSIT_VECTOR, 0)
                            .build();
            //blue left first wobble trajectories
            dropFirstWobbleTrajectories[Alliance.Color.BLUE.ordinal()][StartingPosition.LEFT.ordinal()][RingCount.NONE.ordinal()] =
                    new TrajectoryBuilder(BLUE_LEFT_STARTING_POSE, PhoebeRoadRunnerDrive.getConstraints())
                            .splineToConstantHeading(new Vector2d(-24, -62), 0)
                            .splineToConstantHeading(BLUE_NONE_DEPOSIT_VECTOR, 0)
                            .build();
            dropFirstWobbleTrajectories[Alliance.Color.BLUE.ordinal()][StartingPosition.LEFT.ordinal()][RingCount.ONE.ordinal()] =
                    new TrajectoryBuilder(BLUE_LEFT_STARTING_POSE, PhoebeRoadRunnerDrive.getConstraints())
                            .splineToConstantHeading(new Vector2d(-24, -62), 0)
                            .splineToConstantHeading(BLUE_ONE_DEPOSIT_VECTOR, 0)
                            .build();
            dropFirstWobbleTrajectories[Alliance.Color.BLUE.ordinal()][StartingPosition.LEFT.ordinal()][RingCount.FOUR.ordinal()] =
                    new TrajectoryBuilder(BLUE_LEFT_STARTING_POSE, PhoebeRoadRunnerDrive.getConstraints())
                            .splineToConstantHeading(new Vector2d(-24, -62), 0)
                            .splineToConstantHeading(BLUE_FOUR_DEPOSIT_VECTOR, 0)
                            .build();
            //blue right first wobble trajectories
            dropFirstWobbleTrajectories[Alliance.Color.BLUE.ordinal()][StartingPosition.RIGHT.ordinal()][RingCount.NONE.ordinal()] =
                    new TrajectoryBuilder(BLUE_RIGHT_STARTING_POSE, PhoebeRoadRunnerDrive.getConstraints())
                            .splineToConstantHeading(new Vector2d(-24, 62), 0)
                            .splineToConstantHeading(BLUE_NONE_DEPOSIT_VECTOR, 0)
                            .build();
            dropFirstWobbleTrajectories[Alliance.Color.BLUE.ordinal()][StartingPosition.RIGHT.ordinal()][RingCount.ONE.ordinal()] =
                    new TrajectoryBuilder(BLUE_RIGHT_STARTING_POSE, PhoebeRoadRunnerDrive.getConstraints())
                            .splineToConstantHeading(new Vector2d(-24, 62), 0)
                            .splineToConstantHeading(BLUE_ONE_DEPOSIT_VECTOR, 0)
                            .build();
            dropFirstWobbleTrajectories[Alliance.Color.BLUE.ordinal()][StartingPosition.RIGHT.ordinal()][RingCount.FOUR.ordinal()] =
                    new TrajectoryBuilder(BLUE_RIGHT_STARTING_POSE, PhoebeRoadRunnerDrive.getConstraints())
                            .splineToConstantHeading(new Vector2d(-24, 62), 0)
                            .splineToConstantHeading(BLUE_FOUR_DEPOSIT_VECTOR, 0)
                            .build();
            //blue left first wobble trajectories
            dropFirstWobbleTrajectories[Alliance.Color.BLUE.ordinal()][StartingPosition.LEFT.ordinal()][RingCount.NONE.ordinal()] =
                    new TrajectoryBuilder(BLUE_LEFT_STARTING_POSE, PhoebeRoadRunnerDrive.getConstraints())
                            .splineToConstantHeading(new Vector2d(-24, -62), 0)
                            .splineToConstantHeading(BLUE_NONE_DEPOSIT_VECTOR, 0)
                            .build();
            dropFirstWobbleTrajectories[Alliance.Color.BLUE.ordinal()][StartingPosition.LEFT.ordinal()][RingCount.ONE.ordinal()] =
                    new TrajectoryBuilder(BLUE_LEFT_STARTING_POSE, PhoebeRoadRunnerDrive.getConstraints())
                            .splineToConstantHeading(new Vector2d(-24, -62), 0)
                            .splineToConstantHeading(BLUE_ONE_DEPOSIT_VECTOR, 0)
                            .build();
            dropFirstWobbleTrajectories[Alliance.Color.BLUE.ordinal()][StartingPosition.LEFT.ordinal()][RingCount.FOUR.ordinal()] =
                    new TrajectoryBuilder(BLUE_LEFT_STARTING_POSE, PhoebeRoadRunnerDrive.getConstraints())
                            .splineToConstantHeading(new Vector2d(-24, -60), 0)
                            .splineToConstantHeading(BLUE_FOUR_DEPOSIT_VECTOR, 0)
                            .build();

            //Red-right reach second wobble trajectories
            reachSecondWobbleTrajectories[Alliance.Color.RED.ordinal()][StartingPosition.RIGHT.ordinal()][RingCount.NONE.ordinal()] =
                    new TrajectoryBuilder(new Pose2d(RED_NONE_DEPOSIT_VECTOR, 0), true, PhoebeRoadRunnerDrive.getConstraints())
                            .splineToConstantHeading(new Vector2d(-24, -60), 0)
                            .splineToConstantHeading(new Vector2d(RED_SECOND_WOBBLE_PICK_UP_LEFT.getX(), RED_SECOND_WOBBLE_PICK_UP_LEFT.getX()), 0)
                            .build();
            reachSecondWobbleTrajectories[Alliance.Color.RED.ordinal()][StartingPosition.RIGHT.ordinal()][RingCount.ONE.ordinal()] =
                    new TrajectoryBuilder(new Pose2d(RED_ONE_DEPOSIT_VECTOR, 0), true, PhoebeRoadRunnerDrive.getConstraints())
                            .splineToConstantHeading(new Vector2d(-24, -60), 0)
                            .splineToConstantHeading(new Vector2d(RED_SECOND_WOBBLE_PICK_UP_LEFT.getX(), RED_SECOND_WOBBLE_PICK_UP_LEFT.getX()), 0)
                            .build();
            reachSecondWobbleTrajectories[Alliance.Color.RED.ordinal()][StartingPosition.RIGHT.ordinal()][RingCount.FOUR.ordinal()] =
                    new TrajectoryBuilder(new Pose2d(RED_FOUR_DEPOSIT_VECTOR, 0), true, PhoebeRoadRunnerDrive.getConstraints())
                            .splineToConstantHeading(new Vector2d(-24, -60), 0)
                            .splineToConstantHeading(new Vector2d(RED_SECOND_WOBBLE_PICK_UP_LEFT.getX(), RED_SECOND_WOBBLE_PICK_UP_LEFT.getX()), 0)
                            .build();
            //red-left reach second wobble trajectories
            reachSecondWobbleTrajectories[Alliance.Color.RED.ordinal()][StartingPosition.LEFT.ordinal()][RingCount.NONE.ordinal()] =
                    new TrajectoryBuilder(new Pose2d(RED_NONE_DEPOSIT_VECTOR, 0), true, PhoebeRoadRunnerDrive.getConstraints())
                            .back(48)
                            .build();
            reachSecondWobbleTrajectories[Alliance.Color.RED.ordinal()][StartingPosition.LEFT.ordinal()][RingCount.ONE.ordinal()] =
                    new TrajectoryBuilder(new Pose2d(RED_ONE_DEPOSIT_VECTOR, 0), true, PhoebeRoadRunnerDrive.getConstraints())
                            .back(48)
                            .build();
            reachSecondWobbleTrajectories[Alliance.Color.RED.ordinal()][StartingPosition.LEFT.ordinal()][RingCount.FOUR.ordinal()] =
                    new TrajectoryBuilder(new Pose2d(RED_FOUR_DEPOSIT_VECTOR, 0), true, PhoebeRoadRunnerDrive.getConstraints())
                            .back(48)
                            .build();
            //Blue-right reach second wobble trajectories
            reachSecondWobbleTrajectories[Alliance.Color.BLUE.ordinal()][StartingPosition.RIGHT.ordinal()][RingCount.NONE.ordinal()] =
                    new TrajectoryBuilder(new Pose2d(BLUE_NONE_DEPOSIT_VECTOR, 0), true, PhoebeRoadRunnerDrive.getConstraints())
                            .back(48)
                            .build();
            reachSecondWobbleTrajectories[Alliance.Color.BLUE.ordinal()][StartingPosition.RIGHT.ordinal()][RingCount.ONE.ordinal()] =
                    new TrajectoryBuilder(new Pose2d(BLUE_ONE_DEPOSIT_VECTOR, 0), true, PhoebeRoadRunnerDrive.getConstraints())
                            .back(48)
                            .build();
            reachSecondWobbleTrajectories[Alliance.Color.BLUE.ordinal()][StartingPosition.RIGHT.ordinal()][RingCount.FOUR.ordinal()] =
                    new TrajectoryBuilder(new Pose2d(BLUE_FOUR_DEPOSIT_VECTOR, 0), true, PhoebeRoadRunnerDrive.getConstraints())
                            .back(48)
                            .build();
            //Blue-left reach second wobble trajectories
            reachSecondWobbleTrajectories[Alliance.Color.BLUE.ordinal()][StartingPosition.LEFT.ordinal()][RingCount.NONE.ordinal()] =
                    new TrajectoryBuilder(new Pose2d(BLUE_NONE_DEPOSIT_VECTOR, 0), true, PhoebeRoadRunnerDrive.getConstraints())
                            .back(48)
                            .build();
            reachSecondWobbleTrajectories[Alliance.Color.BLUE.ordinal()][StartingPosition.LEFT.ordinal()][RingCount.ONE.ordinal()] =
                    new TrajectoryBuilder(new Pose2d(BLUE_ONE_DEPOSIT_VECTOR, 0), true, PhoebeRoadRunnerDrive.getConstraints())
                            .back(48)
                            .build();
            reachSecondWobbleTrajectories[Alliance.Color.BLUE.ordinal()][StartingPosition.LEFT.ordinal()][RingCount.FOUR.ordinal()] =
                    new TrajectoryBuilder(new Pose2d(BLUE_FOUR_DEPOSIT_VECTOR, 0), true, PhoebeRoadRunnerDrive.getConstraints())
                            .back(48)
                            .build();

            //Red right drop second wobble trajectories
            dropSecondWobbleTrajectories[Alliance.Color.RED.ordinal()][StartingPosition.RIGHT.ordinal()][RingCount.NONE.ordinal()] =
                    new TrajectoryBuilder(RED_SECOND_WOBBLE_PICK_UP_LEFT, PhoebeRoadRunnerDrive.getConstraints())
                            .splineTo(new Vector2d(12, -50), Math.toRadians(-90))
                            .build();
            dropSecondWobbleTrajectories[Alliance.Color.RED.ordinal()][StartingPosition.RIGHT.ordinal()][RingCount.ONE.ordinal()] =
                    new TrajectoryBuilder(RED_SECOND_WOBBLE_PICK_UP_LEFT, PhoebeRoadRunnerDrive.getConstraints())
                            .splineTo(new Vector2d(RED_ONE_DEPOSIT_VECTOR.getX(), RED_ONE_DEPOSIT_VECTOR.getY()), 0)
                            .build();
            dropSecondWobbleTrajectories[Alliance.Color.RED.ordinal()][StartingPosition.RIGHT.ordinal()][RingCount.FOUR.ordinal()] =
                    new TrajectoryBuilder(RED_SECOND_WOBBLE_PICK_UP_LEFT, PhoebeRoadRunnerDrive.getConstraints())
                            .splineTo(new Vector2d(RED_FOUR_DEPOSIT_VECTOR.getX(), RED_FOUR_DEPOSIT_VECTOR.getY()), 0)
                            .build();

            //Red left drop second wobble trajectories
            dropSecondWobbleTrajectories[Alliance.Color.RED.ordinal()][StartingPosition.LEFT.ordinal()][RingCount.NONE.ordinal()] =
                    new TrajectoryBuilder(RED_SECOND_WOBBLE_PICK_UP_RIGHT, PhoebeRoadRunnerDrive.getConstraints())
                            .splineTo(new Vector2d(RED_NONE_DEPOSIT_VECTOR.getX(), RED_NONE_DEPOSIT_VECTOR.getY()), 0)
                            .build();
            dropSecondWobbleTrajectories[Alliance.Color.RED.ordinal()][StartingPosition.LEFT.ordinal()][RingCount.ONE.ordinal()] =
                    new TrajectoryBuilder(RED_SECOND_WOBBLE_PICK_UP_RIGHT, PhoebeRoadRunnerDrive.getConstraints())
                            .splineTo(new Vector2d(RED_ONE_DEPOSIT_VECTOR.getX(), RED_ONE_DEPOSIT_VECTOR.getY()), 0)
                            .build();
            dropSecondWobbleTrajectories[Alliance.Color.RED.ordinal()][StartingPosition.LEFT.ordinal()][RingCount.FOUR.ordinal()] =
                    new TrajectoryBuilder(RED_SECOND_WOBBLE_PICK_UP_RIGHT, PhoebeRoadRunnerDrive.getConstraints())
                            .splineTo(new Vector2d(RED_FOUR_DEPOSIT_VECTOR.getX(), RED_FOUR_DEPOSIT_VECTOR.getY()), 0)
                            .build();

            //Blue right drop second wobble trajectories
            dropSecondWobbleTrajectories[Alliance.Color.BLUE.ordinal()][StartingPosition.RIGHT.ordinal()][RingCount.NONE.ordinal()] =
                    new TrajectoryBuilder(BLUE_SECOND_WOBBLE_PICK_UP_LEFT, PhoebeRoadRunnerDrive.getConstraints())
                            .splineTo(new Vector2d(BLUE_NONE_DEPOSIT_VECTOR.getX(), BLUE_NONE_DEPOSIT_VECTOR.getY()), 0)
                            .build();
            dropSecondWobbleTrajectories[Alliance.Color.BLUE.ordinal()][StartingPosition.RIGHT.ordinal()][RingCount.ONE.ordinal()] =
                    new TrajectoryBuilder(BLUE_SECOND_WOBBLE_PICK_UP_LEFT, PhoebeRoadRunnerDrive.getConstraints())
                            .splineTo(new Vector2d(BLUE_ONE_DEPOSIT_VECTOR.getX(), BLUE_ONE_DEPOSIT_VECTOR.getY()), 0)
                            .build();
            dropSecondWobbleTrajectories[Alliance.Color.BLUE.ordinal()][StartingPosition.RIGHT.ordinal()][RingCount.FOUR.ordinal()] =
                    new TrajectoryBuilder(BLUE_SECOND_WOBBLE_PICK_UP_LEFT, PhoebeRoadRunnerDrive.getConstraints())
                            .splineTo(new Vector2d(BLUE_FOUR_DEPOSIT_VECTOR.getX(), BLUE_FOUR_DEPOSIT_VECTOR.getY()), 0)
                            .build();

            //Blue left drop second wobble trajectories
            dropSecondWobbleTrajectories[Alliance.Color.BLUE.ordinal()][StartingPosition.LEFT.ordinal()][RingCount.NONE.ordinal()] =
                    new TrajectoryBuilder(BLUE_SECOND_WOBBLE_PICK_UP_RIGHT, PhoebeRoadRunnerDrive.getConstraints())
                            .splineTo(new Vector2d(BLUE_NONE_DEPOSIT_VECTOR.getX(), BLUE_NONE_DEPOSIT_VECTOR.getY()), 0)
                            .build();
            dropSecondWobbleTrajectories[Alliance.Color.BLUE.ordinal()][StartingPosition.LEFT.ordinal()][RingCount.ONE.ordinal()] =
                    new TrajectoryBuilder(BLUE_SECOND_WOBBLE_PICK_UP_RIGHT, PhoebeRoadRunnerDrive.getConstraints())
                            .splineTo(new Vector2d(BLUE_ONE_DEPOSIT_VECTOR.getX(), BLUE_ONE_DEPOSIT_VECTOR.getY()), 0)
                            .build();
            dropSecondWobbleTrajectories[Alliance.Color.BLUE.ordinal()][StartingPosition.LEFT.ordinal()][RingCount.FOUR.ordinal()] =
                    new TrajectoryBuilder(BLUE_SECOND_WOBBLE_PICK_UP_RIGHT, PhoebeRoadRunnerDrive.getConstraints())
                            .splineTo(new Vector2d(BLUE_FOUR_DEPOSIT_VECTOR.getX(), BLUE_FOUR_DEPOSIT_VECTOR.getY()), 0)
                            .build();
            //Red navigation trajectories
            navigationTrajectories[Alliance.Color.RED.ordinal()][RingCount.NONE.ordinal()] =
                    new TrajectoryBuilder(new Pose2d(12, -50, Math.toRadians(-90)), true, PhoebeRoadRunnerDrive.getConstraints())
                            .splineTo(new Vector2d(RED_NONE_DEPOSIT_VECTOR.getX(), -24), Math.toRadians(180))
                            .build();
            navigationTrajectories[Alliance.Color.RED.ordinal()][RingCount.ONE.ordinal()] =
                    new TrajectoryBuilder(new Pose2d(RED_ONE_DEPOSIT_VECTOR.getX(), RED_ONE_DEPOSIT_VECTOR.getY()), true, PhoebeRoadRunnerDrive.getConstraints())
                            .splineTo(new Vector2d(RED_NONE_DEPOSIT_VECTOR.getX(), -36), Math.toRadians(180))
                            .build();
            navigationTrajectories[Alliance.Color.RED.ordinal()][RingCount.FOUR.ordinal()] =
                    new TrajectoryBuilder(new Pose2d(RED_FOUR_DEPOSIT_VECTOR.getX(), RED_FOUR_DEPOSIT_VECTOR.getY()), true, PhoebeRoadRunnerDrive.getConstraints())
                            .splineTo(new Vector2d(RED_NONE_DEPOSIT_VECTOR.getX(), -36), Math.toRadians(180))
                            .build();
            //Blue navigation trajectories
            navigationTrajectories[Alliance.Color.BLUE.ordinal()][RingCount.NONE.ordinal()] =
                    new TrajectoryBuilder(new Pose2d(12, 50, Math.toRadians(-90)), true, PhoebeRoadRunnerDrive.getConstraints())
                            .splineTo(new Vector2d(BLUE_NONE_DEPOSIT_VECTOR.getX(), 36), Math.toRadians(180))
                            .build();
            navigationTrajectories[Alliance.Color.BLUE.ordinal()][RingCount.ONE.ordinal()] =
                    new TrajectoryBuilder(new Pose2d(BLUE_ONE_DEPOSIT_VECTOR.getX(), BLUE_ONE_DEPOSIT_VECTOR.getY()), true, PhoebeRoadRunnerDrive.getConstraints())
                            .splineTo(new Vector2d(BLUE_NONE_DEPOSIT_VECTOR.getX(), 36), Math.toRadians(180))
                            .build();
            navigationTrajectories[Alliance.Color.BLUE.ordinal()][RingCount.FOUR.ordinal()] =
                    new TrajectoryBuilder(new Pose2d(BLUE_FOUR_DEPOSIT_VECTOR.getX(), BLUE_FOUR_DEPOSIT_VECTOR.getY()), true, PhoebeRoadRunnerDrive.getConstraints())
                            .splineTo(new Vector2d(BLUE_NONE_DEPOSIT_VECTOR.getX(), 36), Math.toRadians(180))
                            .build();
            RobotLog.i("SilverTitans: Field set trajectories");
            synchronized (mutex) {
                initialized = true;
                RobotLog.i("SilverTitans: Field initialization completed");
            }
        });
        initThread.start();
    }

    public static Pose2d getStartingPose(Alliance.Color alliance, StartingPosition startingPosition) {
        return startingPoses[alliance.ordinal()][startingPosition.ordinal()];
    }
    public static com.arcrobotics.ftclib.geometry.Pose2d roadRunnerToCameraPose(Pose2d pose) {
        return new com.arcrobotics.ftclib.geometry.Pose2d(pose.getX()*M_PER_INCH, pose.getY()*M_PER_INCH,
                new Rotation2d(pose.getHeading()+Math.PI));
    }
    public static Pose2d cameraToRoadRunnerPose(com.arcrobotics.ftclib.geometry.Pose2d pose) {
        return new Pose2d(pose.getX()/M_PER_INCH, pose.getY()/M_PER_INCH,
                pose.getHeading()+Math.PI);
    }

    public static Trajectory getFirstWobbleGoalDepositTrajectory() {
        Match match = Match.getInstance();
        return dropFirstWobbleTrajectories[match.getAllianceColor().ordinal()][match.getStartingPosition().ordinal()][match.getNumberOfRings().ordinal()];
    }
    public static Trajectory getReachSeonddWobbleGoalTrajectory() {
        Match match = Match.getInstance();
        return reachSecondWobbleTrajectories[match.getAllianceColor().ordinal()][match.getStartingPosition().ordinal()][match.getNumberOfRings().ordinal()];
    }
    public static Trajectory getSecondWobbleGoalDepositTrajectory() {
        Match match = Match.getInstance();
        return dropSecondWobbleTrajectories[match.getAllianceColor().ordinal()][match.getStartingPosition().ordinal()][match.getNumberOfRings().ordinal()];
    }
    public static Trajectory getNavigationTrajectory() {
        Match match = Match.getInstance();
        return navigationTrajectories[match.getAllianceColor().ordinal()][match.getNumberOfRings().ordinal()];
    }
}