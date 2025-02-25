package virtual_robot.robots.classes;

import com.qualcomm.robotcore.hardware.DcMotorExImpl;
import com.qualcomm.robotcore.hardware.configuration.MotorType;

import virtual_robot.controller.BotConfig;
import virtual_robot.controller.VirtualBot;
import virtual_robot.robots.classes.templates.MecanumTemplate;

@BotConfig(name = "CADrivetrain Bot", filename = "cad_drivetrain")
public class CADrivetrain extends MecanumTemplate {
    public CADrivetrain(){
        super(new MecanumConfig(){{
            BL_MOTOR = new VirtualBot.PortConfig(0, HubType.CONTROL_HUB, "backleft");
            FL_MOTOR = new VirtualBot.PortConfig(1, HubType.CONTROL_HUB, "frontleft");
            FR_MOTOR = new VirtualBot.PortConfig(2, HubType.CONTROL_HUB, "frontright");
            BR_MOTOR = new VirtualBot.PortConfig(3, HubType.CONTROL_HUB, "backright");

            MOTOR_TYPE = MotorType.Gobilda192;
            GEAR_RATIO_WHEEL = 1.0;
            DRIVE_WHEEL_DIAMETER = 3.77953;

            WHEEL_BASE_WIDTH = 13.11;
            WHEEL_BASE_LENGTH = 10.72;
            BOT_WIDTH = 15.9;

            L_DEADWHEEL = new VirtualBot.PortConfig(0, HubType.CONTROL_HUB, "backleft");
            R_DEADWHEEL = new VirtualBot.PortConfig(3, HubType.CONTROL_HUB, "backright");
            X_DEADWHEEL = new VirtualBot.PortConfig(1, HubType.CONTROL_HUB, "frontleft");

            DEAD_WHEEL_DIAMETER = 1.25984;
            LEFT_DEADWHEEL_X = -4.796;
            RIGHT_DEADWHEEL_X = 4.796;
            X_DEADWHEEL_Y = 5.694;

            IMU_NAME = "imu";
        }});
    }

    @Override
    protected void createHardwareMap(){
        super.createHardwareMap();

        //Extra here if i want
    }
}
