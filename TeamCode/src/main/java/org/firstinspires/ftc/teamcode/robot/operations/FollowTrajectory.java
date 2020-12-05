package org.firstinspires.ftc.teamcode.robot.operations;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;

import java.util.Locale;

/**
 * An operation to follow the specified road-runner trajectory
 */

public class FollowTrajectory extends Operation {
    private Trajectory trajectory;

    public FollowTrajectory(Trajectory trajectory, String title) {
        this.type = TYPE.FOLLOW_TRAJECTORY;
        this.trajectory = trajectory;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "Trajectory: %s->%s --%s",
                this.trajectory.start().toString(),
                this.trajectory.end().toString(),
                this.title);
    }

    public boolean isComplete(MecanumDriveTrain driveTrain) {
        driveTrain.update();
        Match.log("Trajectory position: " + driveTrain.getPoseEstimate().toString());
        return !driveTrain.isBusy();
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }
}

