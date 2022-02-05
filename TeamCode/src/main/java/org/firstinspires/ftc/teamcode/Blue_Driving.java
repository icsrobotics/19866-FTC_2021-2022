package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@TeleOp(name = "Blue Side Driving", group = "Linear Opmode")
public class Blue_Driving extends LinearOpMode {

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
            rightMotor.setPower(-gamepad1.left_stick_y);
            leftMotor.setPower(-gamepad1.right_stick_y);

            //CODE FOR ARM
            armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            armMotor.setPower(0.66 * gamepad2.left_stick_y);

            //CODE FOR END EFFECTOR
            flippyMotor.setDirection(DcMotorSimple.Direction.FORWARD);

            if (gamepad2.a) {
                flippyMotor.setPower(1.0);

            } else if(gamepad2.y){
                flippyMotor.setPower(-1.0);

            } else if (gamepad2.b) {
                flippyMotor.setPower(0);

            }

            // CODE FOR SERVO
            if (gamepad2.x) {
                carasouelServo.setPosition(0.0);

            } else {
                carasouelServo.setPosition(0.5);

            }
        }
    }
}