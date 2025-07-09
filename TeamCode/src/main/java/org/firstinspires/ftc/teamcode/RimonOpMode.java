package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="rimon_op_mode")
public class RimonOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()){
            MecanumDrive.setDrivePowers( new PoseVelocity2d(
                    new Vector2d(
                            gamepad1.left_stick_x,
                            gamepad1.left_stick_y
                    ),
                    gamepad1.right_stick_x
                    )

            );
            verticalElevators.setPower(gamepad1.right_stick_y);

        }


    }
}
