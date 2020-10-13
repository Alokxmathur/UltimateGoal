package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.components.PickerArm;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robot.operations.CameraOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveForDistanceInDirectionOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveForDistanceOperation;
import org.firstinspires.ftc.teamcode.robot.operations.FoundationGripperOperation;
import org.firstinspires.ftc.teamcode.robot.operations.GyroscopicBearingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.PickerOperation;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeLeftForDistanceOperation;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeLeftForDistanceWithHeadingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeLeftForTimeOperation;
import org.firstinspires.ftc.teamcode.robot.operations.WaitOperation;
import org.firstinspires.ftc.teamcode.robot.operations.WaitUntilVuMarkOperation;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.game.Alliance.Color.BLUE;
import static org.firstinspires.ftc.teamcode.game.Alliance.Color.RED;

public abstract class AutonomousHelper extends OpMode {

    protected Match match;
    protected Robot robot;
    protected Alliance.Color allianceColor;

    public static final long VUFORIA_SETTLING_TIME = 2500; //msecs for vuforia to see image
    public static final double CAUTIOUS_SPEED = 0.6;
    public static final double FAST_SPEED = 1.0;

    protected boolean initialMovementDone, initialMovementQueued;
    protected boolean numberOfRingsDetermined, determinationQueued;
    protected boolean wobbleGoalDeposited, wobbleGoalDepositQueued;
    protected boolean navigated, navigationQueued;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void init(Alliance.Color allianceColor) {
        this.allianceColor = allianceColor;
        this.match = Match.getNewInstance();
        match.init();
        match.setAlliance(allianceColor);
        this.robot = match.getRobot();
        this.robot.init(hardwareMap, telemetry, match, allianceColor);
        robot.queuePrimaryOperation(new CameraOperation(CameraOperation.CameraOperationType.FLASH_ON, "Turn on flash"));

        AutoTransitioner.transitionOnStop(this, "Phoebe: Driver Controlled");
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        if (robot.fullyInitialized()) {
            //keep looking for the rings on the launch stack
            match.setNumberOfRings(robot.getNumberOfRings());
            //update driver station with number of rings
            telemetry.addData("Alliance", allianceColor);
            telemetry.addData("Status", "Ready to autonomous");
            telemetry.addData("Rings", match.getNumberOfRings());
            telemetry.addData("Motors", robot.getMotorStatus());
            telemetry.addData("Picker", robot.getPickerArmStatus());
            telemetry.update();
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
        } else if (!numberOfRingsDetermined) {
            if (!determinationQueued) {
                Match.log("Determining number of rings");
                queueRingDetermination();
                determinationQueued = true;
            }
            numberOfRingsDetermined = robot.operationsCompleted();
        } else if (!wobbleGoalDeposited) {
            if (!wobbleGoalDepositQueued) {
                Match.log("Depositing wobble goal");
                queueWobbleGoalDeposit();
                wobbleGoalDepositQueued = true;
            }
            wobbleGoalDeposited = robot.operationsCompleted();
        } else if (!navigated) {
            if (!navigationQueued) {
                Match.log("Navigating");
                queueNavigation();
                navigationQueued = true;
            }
            navigated = robot.operationsCompleted();
        }
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
        robot.queueTertiaryOperation(new CameraOperation(CameraOperation.CameraOperationType.FLASH_ON, "Turn on flash"));
        //lower foundation gripper so camera can see
        //robot.queueTertiaryOperation(new FoundationGripperOperation(FoundationGripperOperation.OperationType.LOWER, "Lower foundation gripper"));

        //queue following operations on primary thread

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

        //get gripper to position to raise wobble goal
        PickerOperation operationRaise = new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_LIFT,
                "Raise gripper to lift wobble goal");
        robot.queuePrimaryOperation(operationRaise);
    }

    protected void queueRingDetermination() {
        match.setNumberOfRings(robot.getNumberOfRings());
    }

    protected void queueWobbleGoalDeposit() {
        double forwardMovement =0, rightMovement=0;
        int numberOfRingsOnStack = match.getNumberOfRings();
        if (numberOfRingsOnStack == 0) {
            forwardMovement  = 2.5*Field.TILE_WIDTH;
            rightMovement = 1*Field.TILE_WIDTH;
        }
        if (numberOfRingsOnStack == 1) {
            forwardMovement  = 3.5*Field.TILE_WIDTH;
            rightMovement = 0 * Field.TILE_WIDTH;
        }
        if (numberOfRingsOnStack == 4) {
            forwardMovement  = 4.5*Field.TILE_WIDTH;
            rightMovement = 1*Field.TILE_WIDTH;
        }

        robot.queuePrimaryOperation(
                new DriveForDistanceOperation(forwardMovement, CAUTIOUS_SPEED, "Move to the right square"));
        robot.queuePrimaryOperation(
                new StrafeLeftForDistanceOperation(-rightMovement, CAUTIOUS_SPEED, "strafe into the right box"));

        //lower gripper to position to deposit wobble goal
        PickerOperation operationLower = new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_POSITION,
                "Lower gripper to deposit wobble goal");
        operationLower.setShoulderPosition(130);
        robot.queuePrimaryOperation(operationLower);

        //open gripper
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.OPEN_GRIPPER, "Open gripper to release wobble goal"));

        //raise gripper to position to clear wobble goal
        PickerOperation operationClear = new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_POSITION,
                "Raise gripper to clear wobble goal");
        operationClear.setShoulderPosition(300);
        robot.queuePrimaryOperation(operationClear);

    }

    /**
     * Get to the launch line from wherever we had deposited the wobble goal.
     * We don't get back to a fixed location on the field, but on the first tile from the alliance area
     * or the second based on where we were supposed to deposit the wobble goal.
     */
    protected void queueNavigation() {
        double forwardMovement = 0;
        int numberOfRingsOnStack = match.getNumberOfRings();
        if (numberOfRingsOnStack == 0) {
            forwardMovement = .15 * Field.TILE_WIDTH;
        }
        if (numberOfRingsOnStack == 1) {
            forwardMovement = .5 * Field.TILE_WIDTH;
        }
        if (numberOfRingsOnStack == 4) {
            forwardMovement = 1.5 * Field.TILE_WIDTH;
        }

        robot.queuePrimaryOperation(
                new DriveForDistanceOperation(-forwardMovement, CAUTIOUS_SPEED, "Navigate"));
    }
}