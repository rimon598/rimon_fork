package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="mecanum_driving")
public class mechanumDriving extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class,"frontLeft");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class,"backLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class,"frontRight");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class,"backRight");
        waitForStart();
        while (opModeIsActive()){
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rot = gamepad1.right_stick_x;

            double frontLeftPower = y+x+rot;
            double backLeftPower = y-x+rot;
            double frontRightPower = -(y-x-rot);
            double backRightPower = y+x-rot;

            double norm = Math.max(
                    Math.max(Math.abs(frontLeftPower),Math.abs(backLeftPower)),
                    Math.max(Math.abs(frontRightPower),Math.abs(backRightPower))
            );
            if (norm>1){
                frontLeftPower /= norm;
                backLeftPower /= norm;
                frontRightPower /= norm;
                backRightPower /= norm;
            }
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);
        }
    }
}
