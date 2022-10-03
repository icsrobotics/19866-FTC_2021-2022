package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp(name = "Two Wheel Drive", group = "Linear Opmode")
public class Two_Wheel_Drive extends LinearOpMode {
    // 0 -arm, 1 - end, 2 - left, 3 - right
    // Port 0 - left
    DcMotor leftMotor = hardwareMap.dcMotor.get("Back_Left");

    //Port 1 - right
    DcMotor rightMotor = hardwareMap.dcMotor.get("Back_Right");

    DcMotor armMotor = hardwareMap.dcMotor.get("Front_Right");

    DcMotor endMotor = hardwareMap.dcMotor.get("Front_Left");

    @Override public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
            double left  = drive + turn;
            double right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            double max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            leftMotor.setPower(left);
            rightMotor.setPower(right);

            armMotor.setPower(gamepad2.left_stick_y);
            endMotor.setPower(gamepad2.right_stick_y);
        }
    }
}
