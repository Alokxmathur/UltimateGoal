package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;

import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class GyroscopicBearingOperation extends Operation {
    protected double desiredBearing;

    public GyroscopicBearingOperation(double desiredBearing, String title) {
        this.type = TYPE.BEARING;
        this.title = title;
        this.desiredBearing = desiredBearing;
    }

    public String toString() {
        return String.format(Locale.getDefault(),"GyroscopicBearing: %.2f --%s",
                this.desiredBearing, this.title);
    }

    public boolean isComplete(MecanumDriveTrain driveTrain) {
        driveTrain.update();
        return !driveTrain.isBusy();
    }

    public double getDesiredBearing() {
        return desiredBearing;
    }
}
