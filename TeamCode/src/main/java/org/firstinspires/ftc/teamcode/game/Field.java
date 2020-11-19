package org.firstinspires.ftc.teamcode.game;

/**
 * Created by Silver Titans on 9/16/17.
 */

public class Field {
    public static final float MM_PER_INCH = 25.4f;
    public static final float M_PER_INCH = MM_PER_INCH/1000;
    // the width of the FTC field (from the center point to the outer panels)
    public static final double TILE_WIDTH = 24 * MM_PER_INCH;
    public static final double FIELD_WIDTH  = 6 * TILE_WIDTH;
    public static final double TAPE_WIDTH = 2*MM_PER_INCH;

    public enum RingCount {
        ONE, NONE, FOUR
    }

    public enum StartingPosition {
        LEFT, RIGHT
    }
    public Field() {
    }

}