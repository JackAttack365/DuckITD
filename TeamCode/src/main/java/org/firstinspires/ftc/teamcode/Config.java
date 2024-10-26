package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// Config stores everything any of our SubSystems need to function, stores GamePads, Telemetry, HardwareMap,
// and names of each motor
public class Config {

    public Telemetry telemetry;

    public HardwareMap hardwareMap;

    public Gamepad gamePad1;
    public Gamepad gamePad2;

    // Stores the hardwareMap names as constants
    // Drive system
    public static final String RIGHT_FRONT_DRIVE = "rightFront";
    public static final String RIGHT_BACK_DRIVE = "rightBack";
    public static final String LEFT_FRONT_DRIVE = "leftFront";
    public static final String LEFT_BACK_DRIVE = "leftBack";

    public static final String LEFT_LIFT_MOTOR = "leftLift";
    public static final String RIGHT_LIFT_MOTOR = "rightLift";

    public static final int ROBOT_WIDTH = 18;

    // Current game runtime
    private ElapsedTime runtime = new ElapsedTime();

    public double robotX, robotY, robotHeading;

    // Constructor
    public Config(Telemetry tlm, HardwareMap hwm, Gamepad gmp1, Gamepad gmp2) {
        this.telemetry = tlm;
        this.hardwareMap = hwm;
        this.gamePad1 = gmp1;
        this.gamePad2 = gmp2;
    }

    // Telemetry is similar to logging. Appears in Driver Station
    public void updateTelemetry() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.addData("G1: bumper", "L: %b R: %b", gamePad1.left_bumper, gamePad1.right_bumper);
        telemetry.addData("G1: trigger", "L: %4.2f, R: %4.2f", gamePad1.left_trigger, gamePad1.right_trigger);
    }
}