package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.PIDController;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class BearingPIDOperation extends BearingOperation {
    PIDController pidController = new PIDController(1, 0, 0);
    public BearingPIDOperation(double desiredBearing, String title) {
        super(desiredBearing, title);
        pidController.setContinuous();
        pidController.setSetpoint(desiredBearing/180);
    }

    public boolean isComplete(MecanumDriveTrain driveTrain, double currentBearing) {
        // determine turn power based on +/- error
        double error = AngleUnit.normalizeDegrees(desiredBearing - currentBearing) / 180;

        if (Math.abs(error) <= MecanumDriveTrain.HEADING_THRESHOLD) {
            driveTrain.stop();
            return true;
        }
        else {
            pidController.setInput(error);
            double rotation = pidController.performPID();
            driveTrain.drive(0, 0, rotation);
            return false;
        }
    }
}
