package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

@Config
@TeleOp(name = "Cascading Lift", group = "Linear Opmode")
public class Cascading_Lift extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    // Motor variables
    Servo carasouelServo;

    @Override
    public void runOpMode() {

        carasouelServo = hardwareMap.servo.get("Carasouel_Servo"); //initializing carousel
        carasouelServo.setPosition(0.5);
        carasouelServo.setDirection(Servo.Direction.REVERSE);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        //waiting for time to start
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // CODE FOR SERVO
            if (gamepad2.x) {
                carasouelServo.setPosition(0.0);

            } else {
                carasouelServo.setPosition(0.5);

            }
        }
    }
}