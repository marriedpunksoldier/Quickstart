package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.Alliance;

@TeleOp(name = "Red Teleop", group = "Teleop")
public class RedTeleop extends TeleopBase {
    public RedTeleop() {
        super(Alliance.RED);
    }
}
