package org.firstinspires.ftc.teamcode.robot.components.drivetrain;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.roadrunner.drive.PhoebeMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.components.imu.IMU;
import org.firstinspires.ftc.teamcode.robot.operations.ClockwiseRotationOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveForDistanceInDirectionOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveForDistanceOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveForTimeOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveUntilColorOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveUntilVuMarkOperation;
import org.firstinspires.ftc.teamcode.robot.operations.FollowTrajectory;
import org.firstinspires.ftc.teamcode.robot.operations.GyroscopicBearingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.RotateUntilVuMarkOperation;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeLeftForDistanceOperation;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeLeftForDistanceWithHeadingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeLeftForTimeOperation;
import org.firstinspires.ftc.teamcode.robot.operations.TurnOperation;

/**
 * Created by Silver Titans on 10/26/17.
 */

public class MecanumDriveTrain extends PhoebeMecanumDrive {
    //Define constants that help us move appropriate inches based on our drive configuration
    public static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: Rev 20:1 Motor Encoder
    //public static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: Rev 40:1 Motor Encoder
    //public static final double     COUNTS_PER_MOTOR_REV    = 280 ;    // eg: Rev Hex Motor Encoder
    private static final double     WHEEL_RADIUS   = 50;     // For figuring circumference (mm)
    private static  final double     COUNTS_PER_MM = COUNTS_PER_MOTOR_REV  /
            (WHEEL_RADIUS  * Math.PI * 2);
    //our drive train width is 14 7/8 inches
    public static final double DRIVE_TRAIN_WIDTH = 12.125* Field.MM_PER_INCH; //12 1/8 inches
    public static final double DRIVE_TRAIN_LENGTH = 13.125* Field.MM_PER_INCH; //13 1/8 inches
    public static final double ARC_LENGTH_PER_DEGREE = 2 * Math.PI * (Math.hypot(DRIVE_TRAIN_LENGTH, DRIVE_TRAIN_WIDTH) / 2) / 360;
    public static final double TRAVEL_LENGTH_PER_TURN_DEGREE = 2 * Math.PI * (Math.hypot(DRIVE_TRAIN_LENGTH/2, DRIVE_TRAIN_WIDTH)) / 360;

    public static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it
    public static final double     P_TURN_COEFF            = 1;     // Larger is more responsive, but also less stable
    public static final double     P_DRIVE_COEFF           = 0.025;     // Larger is more responsive, but also less stable

    public static final int WITHIN_RANGE = 30;

    // The IMU sensor object
    IMU imu;

    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public MecanumDriveTrain(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap);
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    /** Set power of left motor
     *
     * @param power
     *
     */
    public void setLeftFrontPower(double power) {
        this.leftFront.setPower(power);
    }

    /**
     * Set power of right motor
     * @param power
     */
    public void setRightFrontPower(double power) {
        this.rightFront.setPower(power);
    }

    /** Set power of left motor
     *
     * @param power
     *
     */
    public void setLeftRearPower(double power) {
        this.leftRear.setPower(power);
    }

    /**
     * Set power of right motor
     * @param power
     */
    public void setRightRearPower(double power) {
        this.rightRear.setPower(power);
    }


    public void handleOperation(DriveForTimeOperation operation) {
        stop();
    }

    public void handleOperation(FollowTrajectory trajectoryOperation) {
        super.followTrajectory(trajectoryOperation.getTrajectory());
    }

    /**
     * Handle operation to drive for the specified distance in the direction the robot is facing
     * @param operation
     *
     * We do this by computing how much each wheel must be rotated to travel the specified distance
     * and then commanding the motors to reach the new desired encoder values
     *
     *
     */
    public void handleOperation(DriveForDistanceOperation operation) {
        stop();
        int encoderChange = (int) (operation.getDistance() * COUNTS_PER_MM);
        this.leftFront.setTargetPosition(leftFront.getCurrentPosition() + encoderChange);
        this.rightFront.setTargetPosition(rightFront.getCurrentPosition() + encoderChange);
        this.leftRear.setTargetPosition(leftRear.getCurrentPosition() + encoderChange);
        this.rightRear.setTargetPosition(rightRear.getCurrentPosition() + encoderChange);

        this.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.leftFront.setPower(operation.getSpeed());
        this.rightFront.setPower(operation.getSpeed());
        this.leftRear.setPower(operation.getSpeed());
        this.rightRear.setPower(operation.getSpeed());
    }

    public void handleOperation (DriveForDistanceInDirectionOperation operation) {
        stop();
        int encoderChange = (int) (operation.getDistance() * COUNTS_PER_MM);
        this.leftFront.setTargetPosition(leftFront.getCurrentPosition() + encoderChange);
        this.rightFront.setTargetPosition(rightFront.getCurrentPosition() + encoderChange);
        this.leftRear.setTargetPosition(leftRear.getCurrentPosition() + encoderChange);
        this.rightRear.setTargetPosition(rightRear.getCurrentPosition() + encoderChange);

        this.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void handleOperation (DriveUntilColorOperation operation) {
        stop();
        this.setLeftFrontPower(operation.getSpeed());
        this.setLeftRearPower(operation.getSpeed());
        this.setRightFrontPower(operation.getSpeed());
        this.setRightRearPower(operation.getSpeed());
    }

    /**
     * Handle operation to rotate until a VuMark is seen
     *
     * We simply rotate at the specified SPEED and leave the isComplete check to stop when we see a VuMark
     * @param operation
     */

    public void handleOperation(RotateUntilVuMarkOperation operation) {
        this.leftFront.setPower(operation.getSpeed());
        this.rightFront.setPower(operation.getSpeed());
        this.leftRear.setPower(operation.getSpeed());
        this.rightRear.setPower(operation.getSpeed());
    }

    /**
     * Handle operation to drive until a VuMark is seen
     *
     * We do nothing, leave the isComplete check to stop when we see a VuMark
     * @param operation
     */

    public void handleOperation(DriveUntilVuMarkOperation operation) {
    }

    /**
     * Handle operation to strafe for the specified distance perpendicular to the direction the robot is facing
     * @param operation
     *
     * We do this by computing how much each wheel must be rotated to travel the specified distance
     * and then commanding the motors to reach the new desired encoder values
     *
     * We make the left front and right rear motors move forward while making the right front and left rear
     * motors propel backwards
     *
     * Note that to move the left motors forward, we have to specify an encoder value in the negative direction
     */
    public void handleOperation(StrafeLeftForDistanceOperation operation) {
        stop();
        int encoderChange = (int) (operation.getDistance() * COUNTS_PER_MM * 1.04);
        this.leftFront.setTargetPosition(leftFront.getCurrentPosition() - encoderChange);
        this.rightFront.setTargetPosition(rightFront.getCurrentPosition() + encoderChange);
        this.leftRear.setTargetPosition(leftRear.getCurrentPosition() + encoderChange);
        this.rightRear.setTargetPosition(rightRear.getCurrentPosition() - encoderChange);

        this.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.leftFront.setPower(operation.getSpeed());
        this.rightFront.setPower(operation.getSpeed());
        this.leftRear.setPower(operation.getSpeed());
        this.rightRear.setPower(operation.getSpeed());
    }


    /**
     * Handle operation to strafe for the specified time perpendicular to the direction the robot is facing
     * @param operation
     *
     * We make the left front and right rear motors move forward while making the right front and left rear
     * motors propel backwards
     *
     */
    public void handleOperation(StrafeLeftForTimeOperation operation) {
        stop();
        this.leftFront.setPower(-operation.getSpeed());
        this.rightFront.setPower(operation.getSpeed());
        this.leftRear.setPower(operation.getSpeed());
        this.rightRear.setPower(-operation.getSpeed());
    }
    /**
     * Handle operation to strafe for the specified distance perpendicular to the direction the robot is facing
     * @param operation
     *
     * We do this by computing how much each wheel must be rotated to travel the specified distance
     * and then commanding the motors to reach the new desired encoder values
     *
     * We make the left front and right rear motors move forward while making the right front and left rear
     * motors propel backwards
     *
     */
    public void handleOperation(StrafeLeftForDistanceWithHeadingOperation operation) {
        stop();
        int encoderChange = (int) (operation.getDistance() * COUNTS_PER_MM);
        this.leftFront.setTargetPosition(leftFront.getCurrentPosition() - encoderChange);
        this.rightFront.setTargetPosition(rightFront.getCurrentPosition() + encoderChange);
        this.leftRear.setTargetPosition(leftRear.getCurrentPosition() + encoderChange);
        this.rightRear.setTargetPosition(rightRear.getCurrentPosition() - encoderChange);

        this.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Handle operation to rotate the robot for the specified degrees
     * @param operation
     *
     * We do this by computing how much each wheel must be rotated to travel on the circumference of
     * the circle the wheels would describe if the left ones are propelling the robot forward
     * and the right ones are propelling the robot backwards
     *
     *
     */

    public void handleOperation(ClockwiseRotationOperation operation) {
        stop();

        double arcLength =
                ARC_LENGTH_PER_DEGREE * operation.getDegrees();
        int encoderChange = (int) (arcLength * COUNTS_PER_MM);
        this.leftFront.setTargetPosition(leftFront.getCurrentPosition() + encoderChange);
        this.rightFront.setTargetPosition(rightFront.getCurrentPosition() - encoderChange);
        this.leftRear.setTargetPosition(leftRear.getCurrentPosition() + encoderChange);
        this.rightRear.setTargetPosition(rightRear.getCurrentPosition() - encoderChange);

        this.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.leftFront.setPower(operation.getSpeed());
        this.rightFront.setPower(operation.getSpeed());
        this.leftRear.setPower(operation.getSpeed());
        this.rightRear.setPower(operation.getSpeed());
    }

    public void handleOperation(TurnOperation operation) {
        stop();

        double arcLength =
                TRAVEL_LENGTH_PER_TURN_DEGREE * operation.getDegrees();
        int encoderChange = (int) (arcLength * COUNTS_PER_MM);
        if (operation.getDirection() == TurnOperation.Direction.RIGHT) {
            this.leftFront.setTargetPosition(leftFront.getCurrentPosition() + encoderChange);
            this.leftRear.setTargetPosition(leftRear.getCurrentPosition() + encoderChange);

            this.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            this.leftFront.setPower(operation.getSpeed());
            this.leftRear.setPower(operation.getSpeed());
            this.rightFront.setPower(0);
            this.rightRear.setPower(0);
        }
        else {
            this.rightFront.setTargetPosition(leftFront.getCurrentPosition() + encoderChange);
            this.rightRear.setTargetPosition(leftRear.getCurrentPosition() + encoderChange);

            this.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            this.rightFront.setPower(operation.getSpeed());
            this.rightRear.setPower(operation.getSpeed());
            this.leftFront.setPower(0);
            this.leftRear.setPower(0);
        }
    }

    public void handleOperation(GyroscopicBearingOperation operation) {
        Pose2d currentPose = getPosition();
        Pose2d desiredPose =
                new Pose2d(currentPose.getX(), currentPose.getY(), Math.toRadians(operation.getDesiredBearing()));
        Trajectory trajectory = trajectoryBuilder(currentPose).lineToLinearHeading(desiredPose)
                .build();
        followTrajectoryAsync(trajectory);
    }

    private boolean withinRange(DcMotor... motors) {
        for (DcMotor motor: motors) {
            if (Math.abs(motor.getTargetPosition() - motor.getCurrentPosition()) <= WITHIN_RANGE) {
                return true;
            }
        }
        return false;
    }

    private boolean withinRange()  {

        return withinRange(leftFront, rightFront, leftRear, rightRear);

    }

    public boolean leftWithinRange() {
        if (withinRange(leftRear, leftFront)) {
            stop();
            return true;
        }
        return false;
    }

    public boolean rightWithinRange() {
        if (withinRange(rightRear, rightFront)) {
            stop();
            return true;
        }
        return false;
    }

    /**
     * Check if the drive train is within the specified encoder count
     * @return
     */
    public boolean driveTrainWithinRange() {
        if (withinRange())
        {
            stop();
            return true;
        }
        else {
            return false;
        }
    }

    public void stop() {
        //Stop our motors
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        this.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public String getStatus() {
        return String.format("LF:%.2f(%d>%d),RF:%.2f(%d>%d),LR:%.2f(%d>%d),RR:%.2f(%d>%d)",
            this.leftFront.getPower(), this.leftFront.getCurrentPosition(), this.leftFront.getTargetPosition(),
            this.rightFront.getPower(), this.rightFront.getCurrentPosition(), this.rightFront.getTargetPosition(),
            this.leftRear.getPower(), this.leftRear.getCurrentPosition(), this.leftRear.getTargetPosition(),
            this.rightRear.getPower(), this.rightRear.getCurrentPosition(), this.rightRear.getTargetPosition());
    }

    public static void setMode(DcMotor motor, DcMotor.RunMode mode) {
        motor.setMode(mode);
    }


    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public static double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    /**
     * Drive in the specified direction at the specified speed while rotating at the specified rotation
     * Direction is relative to the robot unless asked for it to be relative to the field
     * @param direction
     * @param speed
     * @param rotation
     * @param fieldRelative
     */
    public void drive(double direction, double speed, double rotation, boolean fieldRelative) {
        final double fieldCentricDirection = (direction
                + (fieldRelative ? getPosition().getHeading() : 0)) % 360;

        double sin =  Math.sin(fieldCentricDirection + Math.PI / 4.0);
        double cos = Math.cos(fieldCentricDirection + Math.PI / 4.0);
        double max = Math.max(Math.abs(sin), Math.abs(cos));
        sin /= max;
        cos /= max;

        double v1 = speed * sin + rotation;
        double v2 = speed * cos - rotation;
        double v3 = speed * cos + rotation;
        double v4 = speed * sin - rotation;

        // Ensure that none of the values go over 1.0. If none of the provided values are
        // over 1.0, just scale by 1.0 and keep all values.
        double scale = max(1.0, v1, v2, v3, v4);
        if (scale > 1) {
            v1 /= scale;
            v2 /= scale;
            v3 /= scale;
            v4 /= scale;
        }
        setLeftFrontPower(-v1);
        setRightFrontPower(v2);
        setLeftRearPower(-v3);
        setRightRearPower(v4);
    }

    /// Maximum absolute value of some number of arguments
    public static double max(double... xs) {
        double ret = 0.0;
        for (double x : xs) {
            ret = Math.max(ret, Math.abs(x));
        }
        return ret;
    }

    public Pose2d getPosition() {
        super.update();
        return super.getPoseEstimate();
    }
}
