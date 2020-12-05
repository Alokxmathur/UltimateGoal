package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robot.operations.BearingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.CameraOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DistanceInDirectionOperation;
import org.firstinspires.ftc.teamcode.robot.operations.FollowTrajectory;
import org.firstinspires.ftc.teamcode.robot.operations.PickerOperation;
import org.firstinspires.ftc.teamcode.robot.operations.WaitOperation;

public abstract class AutonomousHelper extends OpMode {

    public static final int SHOULDER_POSITION_FOR_WOBBLE_JUST_ABOVE_FLOOR = 140;
    public static final int SHOULDER_POSITION_TO_CLEAR_WOBBLE = 200;
    protected Match match;
    protected Robot robot;
    protected Field field;

    public static final double SUPER_CAUTIOUS_SPEED = 0.3;

    protected boolean wobbleLifted, queuedWobbleLift;
    protected boolean numberOfRingsDetermined, determinationQueued;
    protected boolean firstWobbleDeposited, firstWobbleDepositQueued;
    protected boolean navigated, navigationQueued;
    protected boolean returnedForSecondWobble, returnForSecondWobbleQueued;
    protected boolean collectedSecondWobble, collectionOfSecondWobbleQueued;
    protected boolean secondWobbleGoalDeposited, secondWobbleGoalDepositQueued;
    protected boolean poseSet;

    double desiredXForSecondWobble = 47*Field.MM_PER_INCH;
    double desiredYForSecondWobble = Field.TILE_WIDTH + 8*Field.MM_PER_INCH + MecanumDriveTrain.DRIVE_TRAIN_LENGTH/2;

    public int msStuckDetectInit = 10000;
    public int msStuckDetectInitLoop = 10000;
    public int msStuckDetectStart = 10000;
    public int msStuckDetectLoop = 10000;
    public int msStuckDetectStop = 1000;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void init(Alliance.Color allianceColor, Field.StartingPosition startingPosition) {
        AutoTransitioner.transitionOnStop(this, "Phoebe: Driver Controlled");

        this.match = Match.getNewInstance();
        Match.log("Created new match, initializing it");
        match.init();
        Match.log("Match initialized, setting alliance and starting position");
        match.setAlliance(allianceColor);
        match.setStartingPosition(startingPosition);

        this.robot = match.getRobot();
        Match.log("Initializing robot");
        this.robot.init(hardwareMap, telemetry, match);
        Match.log("Robot initialized");
        robot.queueSecondaryOperation(new CameraOperation(CameraOperation.CameraOperationType.FLASH_ON, "Turn on flash"));
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.INITIAL, "Lift wobble goal slightly"));
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        if (!Field.isInitialized()) {
            RobotLog.i("SilverTitans: Waiting for Field to initialize, please wait");
            telemetry.addData("status", "Waiting for Field to initialize, please wait");
            telemetry.update();
        }
        else if (robot.fullyInitialized()) {
            if (!poseSet) {
                Match.log("Setting pose");
                robot.setPose(match.getAllianceColor(), match.getStartingPosition());
                poseSet = true;
            }
            match.updateTelemetry(telemetry,"Initialized, let's go");
        }
        else {
            telemetry.addData("status", "Waiting for vuForia to finish, please wait");
            telemetry.update();
        }
    }
    public void loop() {
        if (!wobbleLifted) {
            if (!queuedWobbleLift) {
                Match.log("Lifting wobble");
                queuedWobbleLift();
                queuedWobbleLift = true;
            }
            wobbleLifted = robot.operationsCompleted();
        } else if (!numberOfRingsDetermined) {
            if (!determinationQueued) {
                Match.log("Determining number of rings");
                queueRingDetermination();
                determinationQueued = true;
            }
            numberOfRingsDetermined = true;
        } else if (!firstWobbleDeposited) {
            if (!firstWobbleDepositQueued) {
                Match.log("Depositing wobble goal");
                queueFirstWobbleGoalDeposit();
                firstWobbleDepositQueued = true;
            }
            firstWobbleDeposited = robot.operationsCompleted();
        } else if (!returnedForSecondWobble) {
            if (!returnForSecondWobbleQueued) {
                Match.log("Grabbing second wobble goal");
                returnForSecondWobble();
                returnForSecondWobbleQueued = true;
            }
            returnedForSecondWobble = robot.operationsCompleted();
        }
        else if (!collectedSecondWobble) {
            if (!collectionOfSecondWobbleQueued) {
                Match.log("Queuing collection for second wobble");
                queueSecondWobbleCollection();
                collectionOfSecondWobbleQueued = true;
            }
            collectedSecondWobble = robot.operationsCompleted();
        }
        else if (!secondWobbleGoalDeposited) {
            if (!secondWobbleGoalDepositQueued) {
                Match.log("Depositing second wobble goal");
                queueDepositSecondWobble();
                secondWobbleGoalDepositQueued = true;
            }
            secondWobbleGoalDeposited = robot.operationsCompleted();
        }
        else if (!navigated) {
            if (!navigationQueued) {
                Match.log("Navigating");
                queueNavigation();
                navigationQueued = true;
            }
            navigated = robot.operationsCompleted();
        }
        match.updateTelemetry(telemetry, "Autonomous");
    }

    protected void queueRingDetermination() {
        match.setNumberOfRings(robot.getNumberOfRings());
    }

    protected void queuedWobbleLift() {
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.HOVER, "Carry"));
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_LIFT, "Lift"));
        //robot.queuePrimaryOperation(new WaitOperation(1000, "Wait"));
    }

    protected void queueFirstWobbleGoalDeposit() {
        robot.queuePrimaryOperation(new FollowTrajectory(Field.getFirstWobbleGoalDepositTrajectory(), "Get to deposit first wobble"));
        //lower gripper to position to deposit wobble goal
        PickerOperation operationLower = new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_POSITION,
                "Lower gripper to deposit wobble goal");
        operationLower.setShoulderPosition(SHOULDER_POSITION_FOR_WOBBLE_JUST_ABOVE_FLOOR);
        robot.queueSecondaryOperation(operationLower);
        //open gripper
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.OPEN_GRIPPER, "Open gripper to release wobble goal"));
    }

    /**
     * Return close to where we started and grab the second wobble goal
     */
    protected void returnForSecondWobble() {
        //raise gripper to position to clear wobble goal
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_LIFT, "Lift"));
        robot.queuePrimaryOperation(new FollowTrajectory(Field.getReachSeonddWobbleGoalTrajectory(), "Reach second wobble"));
    }

    protected void queueSecondWobbleCollection() {
        Match.log("Current position=" + robot.getPosition());
        double currentX = Math.abs(robot.getCurrentX());
        double currentY = Math.abs(robot.getCurrentY());

        double desiredHeading = (match.getStartingPosition() == Field.StartingPosition.RIGHT) ? 90 : -90;

        //correct current x position
        robot.queuePrimaryOperation(
                new DistanceInDirectionOperation(-(desiredXForSecondWobble-currentX), 0, SUPER_CAUTIOUS_SPEED, "Correct distance to wall"));
        robot.queuePrimaryOperation(
                new BearingOperation(desiredHeading, "Face second wobble"));
        //correct current y position
        robot.queuePrimaryOperation(
                new DistanceInDirectionOperation(Math.abs(desiredYForSecondWobble -currentY), desiredHeading, SUPER_CAUTIOUS_SPEED, "Approach wobble"));
        robot.queuePrimaryOperation(
                new BearingOperation(desiredHeading, "Face second wobble"));

        robot.queuePrimaryOperation(new WaitOperation(500, "Wait half a sec"));
        //get gripper to position to grab wobble goal
        PickerOperation operationGrab = new PickerOperation(PickerOperation.PickerOperationType.GRAB,
                "Gripper to lower onto wobble goal");
        robot.queuePrimaryOperation(operationGrab);

        //close gripper
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.CLOSE_GRIPPER, "Close gripper on wobble goal"));
        //robot.queuePrimaryOperation(new WaitOperation(1000, "Wait"));
        //get gripper to position to raise wobble goal
        PickerOperation operationHover = new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_POSITION,
                "Get wobble to be just above floor");
        operationHover.setShoulderPosition(SHOULDER_POSITION_FOR_WOBBLE_JUST_ABOVE_FLOOR);
        robot.queuePrimaryOperation(operationHover);
    }

    /**
     * Deposit second wobble goal exactly where we deposited the first one
     */
    protected void queueDepositSecondWobble() {
        robot.queuePrimaryOperation(new FollowTrajectory(Field.getSecondWobbleGoalDepositTrajectory(), "Deposit second wobble"));
        robot.queuePrimaryOperation(new WaitOperation(500, "Wait half a sec"));
        //deposit second wobble goal
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.OPEN_GRIPPER, "Open gripper to release wobble goal"));

    }

    protected void queueNavigation() {
        //raise gripper to position to clear wobble goal
        PickerOperation operationClear = new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_POSITION,
                "Raise gripper to clear wobble goal");
        operationClear.setShoulderPosition(SHOULDER_POSITION_TO_CLEAR_WOBBLE);
        robot.queueSecondaryOperation(operationClear);
        robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.CLOSE_GRIPPER, "Close gripper"));
        robot.queuePrimaryOperation(new FollowTrajectory(Field.getNavigationTrajectory(), "Navigate"));
    }

    @Override
    public void stop() {
        this.robot.stop();
    }
}