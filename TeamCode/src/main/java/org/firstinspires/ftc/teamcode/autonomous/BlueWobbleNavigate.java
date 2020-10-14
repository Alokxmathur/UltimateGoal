package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="BlueLeft WobbleNav", group="Phoebe")
//@Disabled
public class BlueWobbleNavigate extends AutonomousHelper {

    @Override
    public void init() {
        super.init(Alliance.Color.BLUE, Field.StartingPosition.LEFT);
    }
}