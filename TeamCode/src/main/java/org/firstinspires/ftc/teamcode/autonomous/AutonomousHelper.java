package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.components.PickerArm;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robot.operations.DriveForDistanceInDirectionOperation;
import org.firstinspires.ftc.teamcode.robot.operations.GyroscopicBearingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.PickerOperation;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeLeftForDistanceOperation;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeLeftForDistanceWithHeadingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.WaitOperation;

import static org.firstinspires.ftc.teamcode.game.Alliance.Color.RED;

public abstract class AutonomousHelper extends OpMode {

    public static final int SHOULDER_POSITION_FOR_WOBBLE_JUST_ABOVE_FLOOR = 140;
    public static final int SHOULDER_POSITION_TO_CLEAR_WOBBLE = 200;
    protected Match match;
    protected Robot robot;

    public static final long VUFORIA_SETTLING_TIME = 1500; //msecs for vuforia to see image
    public static final double CAUTIOUS_SPEED = 0.7;
    public static final double SUPER_CAUTIOUS_SPEED = 0.3;

    protected boolean initialMovementDone, initialMovementQueued;
    protected boolean wobbleLifted, queuedWobbleLift;
    protected boolean numberOfRingsDetermined, determinationQueued;
    protected boolean firstWobbleDeposited, firstWobbleDepositQueued;
    protected boolean navigated, navigationQueued;
    protected boolean returnedForSecondWobble, returnForSecondWobbleQueued;
    protected boolean collectedSecondWobble, collectionOfSecondWobbleQueued;
    protected boolean secondWobbleGoalDeposited, secondWobbleGoalDepositQueued;

    double desiredXForSecondWobble = 47*Field.MM_PER_INCH;
    double desiredYForSecondWobble = Field.TILE_WIDTH + 9*Field.MM_PER_INCH + MecanumDriveTrain.DRIVE_TRAIN_LENGTH/2;

    double initialForwardMovement,
            initialLeftMovement = 0,
            secondForwardMovement = 0,
            secondLeftMovement=0,
            backwardsMovementToNavigate = 0,
            leftMovementToNavigate;

    public int msStuckDetectInit = 5000;
    public int msStuckDetectInitLoop = 5000;
    public int msStuckDetectStart = 5000;
    public int msStuckDetectLoop = 5000;
    public int msStuckDetectStop = 1000;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void init(Alliance.Color allianceColor, Field.StartingPosition startingPosition) {
        AutoTransitioner.transitionOnStop(this, "Phoebe: Driver Controlled");

        this.match = Match.getNewInstance();
        match.init();
        match.setAlliance(allianceColor);
        match.setStartingPosition(startingPosition);

        this.robot = match.getRobot();
        this.robot.init(hardwareMap, telemetry, match);
        //robot.queueSecondaryOperation(new CameraOperation(CameraOperation.CameraOperationType.FLASH_ON, "Turn on flash"));
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.INITIAL, "Lift wobble goal slightly"));
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        if (robot.fullyInitialized()) {
            robot.setPose(match.getAllianceColor(), match.getStartingPosition());
            match.updateTelemetry(telemetry,"Initialized, let's go");
        }
        else {
            telemetry.addData("status", "Waiting for vuForia to finish, please wait");
            telemetry.update();
        }
    }
    public void loop() {
        if (!initialMovementDone) {
            //initiate Phoebe
            if (!initialMovementQueued) {
                this.queueInitialOperations();
                initialMovementQueued = true;
            }
            initialMovementDone = robot.primaryOperationsCompleted();
        } else if (!wobbleLifted) {
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
                queueWobbleGoalDeposit();
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
                Match.log("Queuing correction for second wobble");
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

    /**
     * Turn on led lights, lower the foundation gripper on tertiary thread
     * Get gripper to hover position and open it on secondary thread
     * <p>
     * We do all of this in on the secondary thread so that we don't have to wait for them to
     * complete before reaching the bottom sky stone
     */
    protected void queueInitialOperations() {
        //queue following operations on tertiary thread
        //turn on flash
        //
        //robot.queueTertiaryOperation(new CameraOperation(CameraOperation.CameraOperationType.FLASH_ON, "Turn on flash"));
        //lower foundation gripper so camera can see
        //robot.queueTertiaryOperation(new FoundationGripperOperation(FoundationGripperOperation.OperationType.LOWER, "Lower foundation gripper"));

        //queue following operations on primary thread

/*
        //get gripper to position to raise wobble goal
        PickerOperation operationRaise = new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_LIFT,
                "Raise gripper to lift wobble goal");
        robot.queuePrimaryOperation(operationRaise);

        //get gripper to hover position
        PickerOperation operationHover = new PickerOperation(PickerOperation.PickerOperationType.HOVER,
                "Gripper to hover over wobble goal");
        robot.queuePrimaryOperation(operationHover);

        //open gripper
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.OPEN_GRIPPER, "Open gripper"));
        robot.queuePrimaryOperation(new WaitOperation(1000, "Wait"));

        //get gripper to position to grab wobble goal
        PickerOperation operationGrab = new PickerOperation(PickerOperation.PickerOperationType.GRAB,
                "Gripper to lower onto wobble goal");
        robot.queuePrimaryOperation(operationGrab);

        //close gripper
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.CLOSE_GRIPPER, "Close gripper on wobble goal"));
*/
    }

    protected void queueRingDetermination() {
        match.setNumberOfRings(robot.getNumberOfRings());
    }

    protected void queuedWobbleLift() {
        robot.queuePrimaryOperation(
                new PickerOperation(
                        10f*Field.MM_PER_INCH*PickerArm.EXTENSION_ENCODER_COUNT_PER_MM,
                        "Extend"));
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.HOVER, "Carry"));
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_LIFT, "Lift"));
        //robot.queuePrimaryOperation(new WaitOperation(1000, "Wait"));
    }

    protected void queueWobbleGoalDeposit() {
        //Match.log("desired x=" + desiredXForSecondWobble + ", starting=" + VslamCamera.DRIVE_TRAIN_CENTER_START_X);
        initialForwardMovement = 14*Field.MM_PER_INCH;
        if (match.getStartingPosition() == Field.StartingPosition.LEFT) {
            //we are starting from the left starting line
            //scoot left to avoid rings
            initialLeftMovement = .75*Field.TILE_WIDTH;
            if (match.getAllianceColor() == RED) {
                if (match.getNumberOfRings() == Field.RingCount.NONE) {
                    secondLeftMovement = -2*Field.TILE_WIDTH;
                }
                else if (match.getNumberOfRings() == Field.RingCount.ONE) {
                    secondLeftMovement = -1*Field.TILE_WIDTH;
                }
                else {
                    secondLeftMovement = -2*Field.TILE_WIDTH;
                }
            }
            else {
                if (match.getNumberOfRings() == Field.RingCount.NONE) {
                    secondLeftMovement = 0*Field.TILE_WIDTH;
                }
                else if (match.getNumberOfRings() == Field.RingCount.ONE) {
                    secondLeftMovement = -.75*Field.TILE_WIDTH;
                }
                else {
                    secondLeftMovement = 0*Field.TILE_WIDTH;
                }
            }
        }
        else {
            //We are starting from the right starting line

            //scoot right to avoid rings
            initialLeftMovement = -.75*Field.TILE_WIDTH;
            if (match.getAllianceColor() == RED) {
                if (match.getNumberOfRings() == Field.RingCount.NONE) {
                    secondLeftMovement = 0*Field.TILE_WIDTH;
                }
                else if (match.getNumberOfRings() == Field.RingCount.ONE) {
                    secondLeftMovement = 1*Field.TILE_WIDTH;
                }
                else {
                    secondLeftMovement = 0*Field.TILE_WIDTH;
                }
            }
            else {
                if (match.getNumberOfRings() == Field.RingCount.NONE) {
                    secondLeftMovement = 2*Field.TILE_WIDTH;
                }
                else if (match.getNumberOfRings() == Field.RingCount.ONE) {
                    secondLeftMovement = .75*Field.TILE_WIDTH;
                }
                else {
                    secondLeftMovement = 1.75*Field.TILE_WIDTH;
                }
            }
        }
        if (match.getNumberOfRings() == Field.RingCount.NONE) {
            secondForwardMovement = 2*Field.TILE_WIDTH;
            backwardsMovementToNavigate = -.5*Field.TILE_WIDTH;
            if (match.getAllianceColor() == RED) {
                leftMovementToNavigate = 1*Field.TILE_WIDTH;
            }
            else {
                leftMovementToNavigate = -1*Field.TILE_WIDTH;
            }
        }
        else if (match.getNumberOfRings() == Field.RingCount.ONE) {
            secondForwardMovement = 3*Field.TILE_WIDTH;
            backwardsMovementToNavigate = 6*Field.MM_PER_INCH;
            leftMovementToNavigate = 0;
        }
        else {
            secondForwardMovement = 4*Field.TILE_WIDTH;
            backwardsMovementToNavigate = 1*Field.TILE_WIDTH + 6*Field.MM_PER_INCH;
            leftMovementToNavigate = 0;
        }

        robot.queuePrimaryOperation(new DriveForDistanceInDirectionOperation
                (initialForwardMovement, 0, CAUTIOUS_SPEED, "Move forward to clear wall"));
        robot.queuePrimaryOperation(new StrafeLeftForDistanceWithHeadingOperation(
                initialLeftMovement, 0, CAUTIOUS_SPEED, "Move to avoid rings"));
        robot.queuePrimaryOperation(
                new DriveForDistanceInDirectionOperation(secondForwardMovement, 0, CAUTIOUS_SPEED, "Move to the right square"));
        if (secondLeftMovement != 0) {
            robot.queuePrimaryOperation(
                    new StrafeLeftForDistanceWithHeadingOperation(secondLeftMovement, 0, CAUTIOUS_SPEED, "Strafe into the right box"));
        }
        //lower gripper to position to deposit wobble goal
        PickerOperation operationLower = new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_POSITION,
                "Lower gripper to deposit wobble goal");
        operationLower.setShoulderPosition(SHOULDER_POSITION_FOR_WOBBLE_JUST_ABOVE_FLOOR);
        robot.queueSecondaryOperation(operationLower);

        robot.queuePrimaryOperation(new WaitOperation(500, "Wait half a sec"));
        //open gripper
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.OPEN_GRIPPER, "Open gripper to release wobble goal"));
    }

    /**
     * Return close to where we started and grab the second wobble goal
     */
    protected void returnForSecondWobble() {
        //raise gripper to position to clear wobble goal
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_LIFT, "Lift"));

        if (secondLeftMovement != 0) {
            robot.queuePrimaryOperation(
                    new StrafeLeftForDistanceOperation(-secondLeftMovement, CAUTIOUS_SPEED, "Undo strafing into box"));
        }
        robot.queuePrimaryOperation(
                new DriveForDistanceInDirectionOperation(-secondForwardMovement, 0, CAUTIOUS_SPEED, "Move back to second wobble"));
    }

    protected void queueSecondWobbleCollection() {
        double currentX = Math.abs(robot.getCurrentX());
        double currentY = Math.abs(robot.getCurrentY());
        Match.log("Current position=" + currentX + "," + currentY);

        double desiredHeading = (match.getStartingPosition() == Field.StartingPosition.RIGHT) ? 90 : -90;

        //correct current x position
        robot.queuePrimaryOperation(
                new DriveForDistanceInDirectionOperation(-(desiredXForSecondWobble-currentX), 0, SUPER_CAUTIOUS_SPEED, "Correct distance to wall"));
        robot.queuePrimaryOperation(
                new GyroscopicBearingOperation(desiredHeading, "Face second wobble"));
        //correct current y position
        robot.queuePrimaryOperation(
                new DriveForDistanceInDirectionOperation(Math.abs(desiredYForSecondWobble -currentY), desiredHeading, SUPER_CAUTIOUS_SPEED, "Approach wobble"));
        robot.queuePrimaryOperation(
                new GyroscopicBearingOperation(desiredHeading, "Face second wobble"));

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
        //retract from the second wobble goal
        robot.queuePrimaryOperation(
                new DriveForDistanceInDirectionOperation(-Field.TILE_WIDTH, 0, CAUTIOUS_SPEED, "Retract from second wobble"));
        //rotate to face forward
        robot.queuePrimaryOperation(new GyroscopicBearingOperation(0, "Face forward"));
        //drive up to the right square
        robot.queuePrimaryOperation(
                new DriveForDistanceInDirectionOperation(secondForwardMovement-9*Field.MM_PER_INCH, 0, CAUTIOUS_SPEED, "Move forward to deposit second wobble"));
        //strafe into the right square
        robot.queuePrimaryOperation(
                new StrafeLeftForDistanceOperation(secondLeftMovement, CAUTIOUS_SPEED, "Strafe into box"));
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
        if (leftMovementToNavigate != 0) {
            robot.queuePrimaryOperation(
                new StrafeLeftForDistanceWithHeadingOperation(leftMovementToNavigate, 0, CAUTIOUS_SPEED, "Strafe away from wobble goal"));
        }
        //drive back to navigate
        robot.queuePrimaryOperation(
                new DriveForDistanceInDirectionOperation(-backwardsMovementToNavigate, 0, CAUTIOUS_SPEED, "Move to navigate"));
    }
}