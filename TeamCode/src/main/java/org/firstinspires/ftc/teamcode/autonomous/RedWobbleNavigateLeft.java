package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="RedLeft WobbleNav", group="Phoebe")
@Disabled
public class RedWobbleNavigateLeft extends AutonomousHelper {

    @Override
    public void init() {
        super.init(Alliance.Color.RED, Field.StartingPosition.LEFT);
    }
}