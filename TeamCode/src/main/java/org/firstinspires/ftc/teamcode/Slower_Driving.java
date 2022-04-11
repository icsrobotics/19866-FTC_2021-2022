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
    DcMotor leftFront;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    Servo armMotor;

    @Override public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFront = hardwareMap.dcMotor.get("Front_Left");
        backLeft = hardwareMap.dcMotor.get("Back_Right");
        frontRight = hardwareMap.dcMotor.get("Front_Right");
        backRight = hardwareMap.dcMotor.get("Back_Left");

        armMotor = hardwareMap.servo.get("Arm_Motor"); // initializing arm motor

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        //waiting for time to start
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            //TANK MODE FOR DRIVING
            leftFront.setPower(gamepad1.left_stick_y * 0.5);
            backLeft.setPower(gamepad1.left_stick_y * 0.5);

            frontRight.setPower(-gamepad1.right_stick_y * 0.5);
            backRight.setPower(-gamepad1.right_stick_y * 0.5);

            //CODE FOR ARM
            if (gamepad2.dpad_up) {
                armMotor.setPosition(1.0);
            } else if (gamepad2.dpad_down){
                armMotor.setPosition(0.0);
            } else{
                armMotor.setPosition(0.5);
            }
        }
    }
}