package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

@Config
@TeleOp(name = "Driving ඞ", group = "Linear Opmode")
public class Slower_Driving extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    // Motor variables
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor armMotor;
    DcMotor flippyMotor;
    Servo carasouelServo;

    @Override public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftMotor = hardwareMap.dcMotor.get("Left_Motor"); // initializing left motor
        rightMotor = hardwareMap.dcMotor.get("Right_Motor"); // initializing right motor
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor = hardwareMap.dcMotor.get("Arm_Motor"); // initializing arm motor

        flippyMotor = hardwareMap.dcMotor.get("Flippy_Motor"); //initializing end effector (i wonder if this will show up)

        carasouelServo = hardwareMap.servo.get("Carasouel_Servo"); //initializing carousel
        carasouelServo.setPosition(0.5);
        carasouelServo.setDirection(Servo.Direction.REVERSE);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        //waiting for time to start
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            //TANK MODE FOR DRIVING
            rightMotor.setPower(-gamepad1.left_stick_y * 0.5);
            leftMotor.setPower(-gamepad1.right_stick_y * 0.5);

            //CODE FOR ARM
            armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            armMotor.setPower(0.7 * gamepad2.left_stick_y);

        }
    }
}