package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.Alliance;

@TeleOp(name = "Blue Teleop", group = "Teleop")
public class BlueTeleop extends TeleopBase {
    public BlueTeleop() {
        super(Alliance.BLUE);
    }
}
