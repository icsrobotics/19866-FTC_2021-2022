package org.firstinspires.ftc.teamcode.BlueAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "Blue Side (Near Carousel) Without Going To Warehouse", group = "Linear Opmode")
public class Auto_Blue_1 extends LinearOpMode {
    // Camera variables
    private OpenCvCamera webcam;
    SkystoneDeterminationPipeline pipeline;
    static double elementPosition;

    // Encoder variables
    private final ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV    = 1440;
    static final double DRIVE_GEAR_REDUCTION    = 2.0;
    static final double WHEEL_DIAMETER_INCHES   = 4.0;
    static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double ARM_DRIVE_GEAR_REDUCTION = 6;
    static final double ARM_COUNTS_PER_MOTOR     = 288;
    static final double ARM_COUNTS_PER_INCH      = ARM_DRIVE_GEAR_REDUCTION * ARM_COUNTS_PER_MOTOR;

    // Motor variables
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor armMotor;
    Servo   carasouelServo;
    DcMotor flippyMotor;

    @Override
    public void runOpMode() {
        // initializing all camera elements
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,
                "Webcam 1"), cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() {
                webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override public void onError(int errorCode) {
                telemetry.addData("Camera", "Unavailable");
                telemetry.update();
            }
        });

        // initializing all motor elements
        leftMotor = hardwareMap.dcMotor.get("Right_Motor");
        rightMotor = hardwareMap.dcMotor.get("Left_Motor");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // initializing arm motor
        armMotor = hardwareMap.dcMotor.get("Arm_Motor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //initializing end effector
        flippyMotor = hardwareMap.dcMotor.get("Flippy_Motor");
        // initializing carousel
        carasouelServo = hardwareMap.servo.get("Carasouel_Servo");
        carasouelServo.setPosition(0.5);
        carasouelServo.setDirection(Servo.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        // FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        FtcDashboard.getInstance().startCameraStream(webcam, 10);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // go to shipping hub
        encoderDrive(0.15, 5, 5, 0.5);
        encoderDrive(0.25, -1, 1, 0.65);
        encoderDrive(0.135, 5, 5, 0.85);

        // lift arm
        ArmLift(0.5, 10, 3.3);
        encoderDrive(0.3, 5, 5, 0.3);
        sleep(50);
        flippyMotor.setPower(1.0);
        sleep(3000);
        flippyMotor.setPower(0.0);

        // go to warehouse
        encoderDrive(0.3,-1, -1, 0.5);
        encoderDrive(0.2, -2, 2, 0.75);
        ArmLift(1.0, 10, 0.5);
        encoderDrive(0.15, 10, 10, 2.3);

        if (elementPosition == 3) /* RIGHT - highest level */ {

            // go to shipping hub
            encoderDrive(0.15, 5, 5, 0.5);
            encoderDrive(0.25, -1, 1, 0.65);
            encoderDrive(0.135, 5, 5, 0.85);

            // lift arm
            ArmLift(0.5, 10, 3.3);
            encoderDrive(0.3, 5, 5, 0.3);
            sleep(50);
            flippyMotor.setPower(1.0);
            sleep(3000);
            flippyMotor.setPower(0.0);

            // back up
            encoderDrive(0.3,-1, -1, 0.5);

            return;
        } else if (elementPosition == 2) /* CENTER */ {

            // go to shipping hub
            encoderDrive(0.15, 5, 5, 0.5);
            encoderDrive(0.25, -1, 1, 0.65);
            encoderDrive(0.135, 5, 5, 0.85);

            // lift arm
            ArmLift(0.5, 10, 2.5);
            encoderDrive(0.3, 5, 5, 0.3);
            sleep(50);
            flippyMotor.setPower(1.0);
            sleep(3000);
            flippyMotor.setPower(0.0);

            // back up
            encoderDrive(0.3,-1, -1, 0.5);
            encoderDrive(0.2, -2, 2, 0.75);
            ArmLift(1.0, 10, 0.5);
            encoderDrive(0.15, 10, 10, 2.3);

            return;

        } else if (elementPosition == 1) /* LEFT - lowest level */ {

            // go to shipping hub
            encoderDrive(0.15, 5, 5, 0.5);
            encoderDrive(0.25, -1, 1, 0.65);
            encoderDrive(0.135, 5, 5, 0.85);

            // lift arm
            ArmLift(0.5, 10, 2.0);
            encoderDrive(0.3, 5, 5, 0.3);
            sleep(50);
            flippyMotor.setPower(1.0);
            sleep(3000);
            flippyMotor.setPower(0.0);

            // back up
            encoderDrive(0.3,-1, -1, 0.5);

            return;

        } else {
            telemetry.addData("Shipping Element", "Unavailable");
            telemetry.update();
        }

        while (opModeIsActive()) {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline {
        /*
         * An enum to define the skystone position
         */
        public enum SkystonePosition
        {
            LEFT,
            CENTER,
            RIGHT
        }

        /*
         * Some color constants
         */
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar WHITE = new Scalar(255, 255, 255);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(100,250);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(300,250);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(500,250);
        static final int REGION_WIDTH = 100;
        static final int REGION_HEIGHT = 100;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb, region2_Cb, region3_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1, avg2, avg3;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile SkystonePosition position = SkystonePosition.LEFT;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];

            /*
             * The following rectangles are simply visual aid. They serve no functional purpose.
             */

            //region 1
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    WHITE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            // region 2
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    WHITE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            // region 3
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    WHITE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            /*
             * Find the max of the 3 averages
             */
            int maxOneTwo = Math.max(avg1, avg2);
            int max = Math.max(maxOneTwo, avg3);

            /*
             * Now that we found the max, we actually need to go and
             * figure out which sample region that value was from
             */
            if(max == avg1) // Was it from region 1?
            {
                elementPosition = 1; // for use in the encoders, LEFT
                position = SkystonePosition.LEFT; // Record our analysis

                //overlays a green rectangle
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        2); // Negative thickness means solid fill (Which we are not using)
            }
            else if(max == avg2) // Was it from region 2?
            {
                elementPosition = 2; //CENTER
                position = SkystonePosition.CENTER; // Record our analysis

                // overlays a green rectangle
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        2); // Negative thickness means solid fill
            }
            else if(max == avg3) // Was it from region 3?
            {
                elementPosition = 3; //RIGHT
                position = SkystonePosition.RIGHT; // Record our analysis

                // overlays a green rectangle
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region3_pointA, // First point which defines the rectangle
                        region3_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        2); // Negative thickness means solid fill
            }

            return input;
        }
        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public SkystonePosition getAnalysis()
        {
            return position;
        }
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeout) {
        int newLeftTarget;
        int newRightTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftTarget = leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(Math.abs(speed));
            rightMotor.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeout) && (leftMotor.isBusy() || rightMotor.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Original Path",  "Running to %7d : %7d", newLeftTarget, newRightTarget);
                telemetry.addData("Running Now",  "Running at %7d :%7d",
                        leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                telemetry.update();
            }
            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void ArmLift(double speed, double Inches, double timeout) {
        int newTarget;

        // Ensure that the opmode is still active int()
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newTarget = armMotor.getCurrentPosition() + (int)(Inches * ARM_COUNTS_PER_INCH);
            armMotor.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(Math.abs(speed + 0.07));

            while (opModeIsActive() && (runtime.seconds() < timeout) && armMotor.isBusy()) {
                // Display it for the driver.
                telemetry.addData("Original Path",  "Running to %7d", newTarget);
                telemetry.addData("Running Now",  "Running at %7d", armMotor.getCurrentPosition());
                telemetry.update();
            }
            // Stop all motion
            armMotor.setPower(0.0);

            // Turn off RUN_TO_POSITION
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void JustRight(double speed, double Inches, double timeout) {
        int newTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newTarget = leftMotor.getCurrentPosition() + (int)(Inches * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeout) && (leftMotor.isBusy() || rightMotor.isBusy())) {
                telemetry.addData("From ICS Robotics", "JUST LEFT!!");
                telemetry.update();
            }
            // Stop all motion;
            leftMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void JustLeft(double speed, double Inches, double timeout) {
        int newTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newTarget = rightMotor.getCurrentPosition() + (int)(Inches * COUNTS_PER_INCH);
            rightMotor.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            rightMotor.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeout) && (leftMotor.isBusy() || rightMotor.isBusy())) {
                telemetry.addData("From ICS Robotics", "JUST RIGHT!!");
                telemetry.update();
            }
            // Stop all motion;
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

}
