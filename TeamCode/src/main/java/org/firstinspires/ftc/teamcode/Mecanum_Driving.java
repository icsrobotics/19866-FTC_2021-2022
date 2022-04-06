package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "ඞ", group = "Linear Opmode")

public class Mecanum_Driving extends LinearOpMode {

    Servo servo;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftFront = hardwareMap.dcMotor.get("Front_Left");
        DcMotor backLeft = hardwareMap.dcMotor.get("Back_Left");
        DcMotor frontRight = hardwareMap.dcMotor.get("Front_Right");
        DcMotor backRight = hardwareMap.dcMotor.get("Back_Right");


        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            servo = hardwareMap.servo.get("Carasouel_Servo"); //initializing carousel
            servo.setPosition(0.5);
            servo.setDirection(Servo.Direction.REVERSE);

            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;


            leftFront.setPower(v1);
            frontRight.setPower(v2);
            backLeft.setPower(v3);
            backRight.setPower(v4);

            // Strafe left
            leftFront.setPower(-gamepad1.left_trigger);
            backRight.setPower(-gamepad1.left_trigger);

            frontRight.setPower(gamepad1.left_trigger * 0.6);
            backLeft.setPower(gamepad1.left_trigger * 0.6);

            // Strafe right
            leftFront.setPower(gamepad1.right_trigger * 0.6);
            backRight.setPower(gamepad1.right_trigger * 0.6);

            frontRight.setPower(-gamepad1.right_trigger);
            backLeft.setPower(-gamepad1.right_trigger);

        }
    }
}