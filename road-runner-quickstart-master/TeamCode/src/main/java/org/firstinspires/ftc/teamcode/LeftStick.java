package org.firstinspires.ftc.teamcode;


import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name = "UpdatedLeftStick")
public class LeftStick extends OpMode {
    public DcMotor leftFront, leftRear, rightRear, rightFront;
    public DcMotor armRotation, armRevolution, carousel;
    public CRServo intake;
    public Servo index;
    public ElapsedTime runtime = new ElapsedTime();

    // above initializes all the aspects we need to make our robot function
    @Override
    public void init() {
        telemetry.clearAll();
        // defining all the hardware
        //Wheels
        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        leftRear = hardwareMap.get(DcMotorEx.class, "lr");
        rightRear = hardwareMap.get(DcMotorEx.class, "rr");
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");
        //Carousel
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        //Arm
        armRotation = hardwareMap.get(DcMotor.class, "armRot");
        armRevolution = hardwareMap.get(DcMotor.class, "armRev");
        intake = hardwareMap.get(CRServo.class, "intake");
        index = hardwareMap.get(Servo.class, "index");
        //Reversing inverted motors
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addLine( "Ready to Deploy Teleop");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.clearAll();
        telemetry.addLine( "Running Teleop");
        telemetry.update();

        double leftFrontPower = (0.8 * gamepad1.left_stick_y);
        double leftRearPower = (0.8 * gamepad1.right_stick_x);
        double rightFrontPower = (0.8 * gamepad1.right_stick_x);
        double rightRearPower = (0.8 * gamepad1.left_stick_y);

        if (gamepad1.left_trigger > 0.1) {
            leftFrontPower = -0.75 * gamepad1.left_trigger;
            rightRearPower = 0.75 * gamepad1.left_trigger;
        }

        if (gamepad1.right_trigger > 0.1) {
            leftFrontPower = 0.75 * gamepad1.right_trigger;
            rightRearPower = -0.75 * gamepad1.right_trigger;
        }

        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);

        //Hold X motor position when no power supplied
        if (gamepad1.left_stick_y <= 0.1 && gamepad1.left_trigger <= 0.1 && gamepad1.right_trigger <= 0.1) {
            leftFront.setPower(0);
            rightRear.setPower(0);
        }

        //Hold Y motor position when no power supplied
        if (gamepad1.right_stick_x <= 0.1 && gamepad1.left_trigger <= 0.1 && gamepad1.right_trigger <= 0.1) {
            rightFront.setPower(0);
            leftRear.setPower(0);
        }

        if (gamepad2.y) {
            armRotation.setPower(0);
        }

        armRevolution.setPower(gamepad2.right_stick_x/1.25);
        armRotation.setPower(-gamepad2.right_stick_y/2);


        if (gamepad1.dpad_down){
            telemetry.addLine( "Carousel Entered Slow");
            telemetry.update();
            runtime.reset();
            while (runtime.seconds() <= 1.5 && gamepad1.dpad_down) {
                carousel.setPower(-0.5);
            }
            telemetry.addLine( "Carousel Entered Fast");
            telemetry.update();
            runtime.reset();
            while (runtime.seconds() <= 1.0 && gamepad1.dpad_down) {
                carousel.setPower(-1);
            }
            telemetry.clear();
            telemetry.clear();
        } else if (gamepad1.dpad_up){
            telemetry.addLine( "Carousel Entered Slow");
            telemetry.update();
            runtime.reset();
            while (runtime.seconds() <= 1.5 && gamepad1.dpad_up) {
                carousel.setPower(0.5);
            }
            telemetry.addLine( "Carousel Entered Fast");
            telemetry.update();
            runtime.reset();
            while (runtime.seconds() <= 1.0 && gamepad1.dpad_up) {
                carousel.setPower(1);
            }
            telemetry.clear();
            telemetry.clear();
        } else {
            if (gamepad2.dpad_down){
                telemetry.addLine( "Carousel Entered Slow");
                telemetry.update();
                runtime.reset();
                while (runtime.seconds() <= 1.5 && gamepad2.dpad_down) {
                    carousel.setPower(-0.5);
                }
                telemetry.addLine( "Carousel Entered Fast");
                telemetry.update();
                runtime.reset();
                while (runtime.seconds() <= 1.0 && gamepad2.dpad_down) {
                    carousel.setPower(-1);
                }
                telemetry.clear();
                telemetry.clear();
            } else if (gamepad2.dpad_up){
                telemetry.addLine( "Carousel Entered Slow");
                telemetry.update();
                runtime.reset();
                while (runtime.seconds() <= 1.5 && gamepad2.dpad_up) {
                    carousel.setPower(0.5);
                }
                telemetry.addLine( "Carousel Entered Fast");
                telemetry.update();
                runtime.reset();
                while (runtime.seconds() <= 1.0 && gamepad2.dpad_up) {
                    carousel.setPower(1);
                }
                telemetry.clear();
                telemetry.clear();
            } else{
                carousel.setPower(0);
            }
        }

        if (gamepad2.a) {
            telemetry.addLine( "Opening Index");
            telemetry.update();
            index.setPosition(0.7);
            telemetry.clear();
        } else {
            index.setPosition(0);
        }

        if (gamepad2.x)
        {
            telemetry.addLine( "Intake Running");
            telemetry.update();
            intake.setPower(1);
            telemetry.clear();
        } else if (gamepad2.b) {
            telemetry.addLine( "Output Running");
            telemetry.update();
            intake.setPower(-1);
            telemetry.clear();
        } else {
            intake.setPower(0);
        }

    }
    @Override
    public void stop()
    {
    }
}

