package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

@TeleOp(name="ScrimTeleOp", group = "TeleOp")
public class ScrimTeleOp extends OpMode {
    Config config = null;
    Drivetrain drive = null;
    Arm arm = null;
    Intake intake = null;

    @Override
    public void init() {
        config = new Config(telemetry,hardwareMap,gamepad1,gamepad2);
        drive = new Drivetrain(config);
        arm = new Arm(config);
        intake = new Intake(config);

        drive.init();
        arm.init();
        intake.init();
    }

    @Override
    public void loop() {
        drive.update();
        arm.update();
        intake.update();

        config.updateTelemetry();
    }
}
