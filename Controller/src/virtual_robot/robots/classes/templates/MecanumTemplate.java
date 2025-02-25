package virtual_robot.robots.classes.templates;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.robotcore.hardware.DcMotorControllerImpl;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorExImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeadWheelEncoder;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.MotorType;

import org.dyn4j.dynamics.Body;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Vector2;
import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import virtual_robot.config.Config;
import virtual_robot.controller.Filters;
import virtual_robot.controller.VirtualBot;
import virtual_robot.controller.VirtualField;
import virtual_robot.util.AngleUtils;

/**
 * For internal use only. Represents a robot with four mecanum wheels, a BNO055IMU, and three dead-
 * wheel encoder pods.
 *
 */
public class MecanumTemplate extends VirtualBot {

    protected static class MecanumConfig{
        //Dimensions in inches for encoder wheels.
        //Right and left encoder wheels are oriented parallel to robot-Y axis (i.e., fwd-reverse)
        //X Encoder wheel is oriented parallel to the robot-X axis (i.e., right-left axis)
        protected double DEAD_WHEEL_DIAMETER;
        //Distances of right and left encoder wheels from robot centerline (i.e., the robot-X coordinates of the wheels)
        protected double LEFT_DEADWHEEL_X;
        protected double RIGHT_DEADWHEEL_X;
        //Distance of X-Encoder wheel from robot-X axis (i.e., the robot-Y coordinate of the wheel)
        protected double X_DEADWHEEL_Y;

        protected PortConfig BL_MOTOR;
        protected PortConfig FL_MOTOR;
        protected PortConfig FR_MOTOR;
        protected PortConfig BR_MOTOR;

        protected PortConfig R_DEADWHEEL;
        protected PortConfig L_DEADWHEEL;
        protected PortConfig X_DEADWHEEL;

        protected String IMU_NAME;

        protected MotorType MOTOR_TYPE;

        /*
         * Gear ratio for any external gears added in drive train. For now, this is just 1.0. Could easily
         * add a constructor to MecanumPhysicsBase that allows this to be set to some other value.
         */
        protected double GEAR_RATIO_WHEEL;

        protected double BOT_WIDTH;
        protected double DRIVE_WHEEL_DIAMETER;
        protected double WHEEL_BASE_WIDTH;
        protected double WHEEL_BASE_LENGTH;
    }

    private final MecanumConfig config;

    private DeadWheelEncoder rightEncoder = null;
    private DeadWheelEncoder leftEncoder = null;
    private DeadWheelEncoder xEncoder = null;

    private Integer[] redirect = new Integer[3];

    //Dimensions in pixels -- to be determined in the constructor
    private double encoderWheelRadius;
    private double leftEncoderX;
    private double rightEncoderX;
    private double xEncoderY;

    private DcMotorExImpl[] motors = null;
    private BNO055IMUImpl imu = null;
    BNO055IMUNew imuNew = null;

    /*
     * Robot geometry, in pixels. These will be calculated in the initialize() method. They cannot be computed
     * here, because botwidth is not known until the time of construction.
     */
    private double wheelCircumference;
    private double interWheelWidth;
    private double interWheelLength;
    private double wlAverage;

    /*
     * Transform from wheel motion to robot motion (KINETIC MODEL). This will be computed in the initialize()
     * method, after basic robot geometry is computed.
     */
    private double[][] tWR;

    GeneralMatrixF M_ForceWheelToRobot; // Converts from individual wheel forces to total force/torque on robot
    MatrixF M_ForceRobotToWheel;  // Converts from total force/torque on robot to individual wheel forces
    protected float maxWheelXForce; // Need to assign this in the initialize method.

    /**
     * No-param constructor. Uses the default motor type of Neverest 40
     */
    protected MecanumTemplate(MecanumConfig config) {
        super(config.BOT_WIDTH);
        this.config = config;
    }

    public void initialize(){
        super.initialize();
        hardwareMap.setActive(true);

        motors = new DcMotorExImpl[]{
                (DcMotorExImpl) hardwareMap.get(DcMotorEx.class, config.BL_MOTOR.PORT_NAME),
                (DcMotorExImpl) hardwareMap.get(DcMotorEx.class, config.FL_MOTOR.PORT_NAME),
                (DcMotorExImpl) hardwareMap.get(DcMotorEx.class, config.FR_MOTOR.PORT_NAME),
                (DcMotorExImpl) hardwareMap.get(DcMotorEx.class, config.BR_MOTOR.PORT_NAME)
        };

        imu = hardwareMap.get(BNO055IMUImpl.class, config.IMU_NAME);
        imuNew = hardwareMap.get(BNO055IMUNew.class, config.IMU_NAME);

        double pixelsPerInch = botWidth/config.BOT_WIDTH;

        wheelCircumference = pixelsPerInch * Math.PI * config.DRIVE_WHEEL_DIAMETER;
        interWheelWidth = pixelsPerInch * config.WHEEL_BASE_WIDTH;
        interWheelLength = pixelsPerInch * config.WHEEL_BASE_LENGTH;
        wlAverage = (interWheelLength + interWheelWidth) / 2.0;

        tWR = new double[][]{
                {-0.25, 0.25, -0.25, 0.25},
                {0.25, 0.25, 0.25, 0.25},
                {-0.25 / wlAverage, -0.25 / wlAverage, 0.25 / wlAverage, 0.25 / wlAverage},
                {-0.25, 0.25, 0.25, -0.25}
        };

        float RRt2 = 0.5f * (float)Math.sqrt(interWheelLength*interWheelLength + interWheelWidth*interWheelWidth)
                * (float)Math.sqrt(2.0) / (float) VirtualField.PIXELS_PER_METER;

        /*
         * Converts from the frictional X-component forces at each wheel to the X, Y forces, Torque, and "Fail"
         * forces on the robot. (in the ROBOT coordinate system)
         */
        M_ForceWheelToRobot = new GeneralMatrixF(4, 4, new float[]{
                1, 1, 1, 1,
                -1, 1, -1, 1,
                RRt2, -RRt2, -RRt2, RRt2,
                1, 1, -1, -1});

        /*
         * Converts from X & Y Forces, Torque, and "Fail" force on robot to the corresponding forces (X-component)
         * at each wheel. (in the ROBOT coordinate system)
         */
        M_ForceRobotToWheel = M_ForceWheelToRobot.inverted();

        // Maximum possible frictional force (in robot-X direction) between field and any individual robot wheel.
        // Note the division by 4 (assumes each wheel gets 1/4 of robot mass) and the division by sqrt(2) (because
        // the X-direction force is 1/sqrt(2) times the total friction force on the wheel.
        maxWheelXForce = (float)(9.8 * chassisBody.getMass().getMass()
                * Config.FIELD_FRICTION_COEFF / (4.0 * Math.sqrt(2)));

        // THIS IS STUPID BUT DOES REDIRECTION OF ENCODERS
        if(redirect[1]==null){
            leftEncoder = hardwareMap.get(DeadWheelEncoder.class, "enc_left");
        } else {
            PortConfig encConfig = config.L_DEADWHEEL;
            DcMotorControllerImpl controller = encConfig.HUB_TYPE == HubType.CONTROL_HUB ? motorController0 : motorController1;
            leftEncoder = new DeadWheelEncoder(config.MOTOR_TYPE, controller, encConfig.PORT_NUM);
            motors[redirect[1]].replaceEncInput(leftEncoder);
        }
        if(redirect[0]==null) {
            rightEncoder = hardwareMap.get(DeadWheelEncoder.class, "enc_right");
        } else{
            PortConfig encConfig = config.R_DEADWHEEL;
            DcMotorControllerImpl controller = encConfig.HUB_TYPE == HubType.CONTROL_HUB ? motorController0 : motorController1;
            rightEncoder = new DeadWheelEncoder(config.MOTOR_TYPE, controller, encConfig.PORT_NUM);
            motors[redirect[0]].replaceEncInput(rightEncoder);
        }

        if(redirect[2]==null) {
            xEncoder = hardwareMap.get(DeadWheelEncoder.class, "enc_x");
        } else{
            PortConfig encConfig = config.X_DEADWHEEL;
            DcMotorControllerImpl controller = encConfig.HUB_TYPE == HubType.CONTROL_HUB ? motorController0 : motorController1;
            xEncoder = new DeadWheelEncoder(config.MOTOR_TYPE, controller, encConfig.PORT_NUM);
            motors[redirect[2]].replaceEncInput(xEncoder);
        }

        rightEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
        leftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        xEncoder.setDirection(DcMotorSimple.Direction.FORWARD);

        //Dimensions in pixels
        encoderWheelRadius = pixelsPerInch * 0.5 * config.DEAD_WHEEL_DIAMETER;
        leftEncoderX = pixelsPerInch * config.LEFT_DEADWHEEL_X;
        rightEncoderX = pixelsPerInch * config.RIGHT_DEADWHEEL_X;
        xEncoderY = pixelsPerInch * config.X_DEADWHEEL_Y;

        hardwareMap.setActive(false);
    }

    /**
     * Create the hardware map for this robot. This will include the drive motors, distance sensors, BNO055IMU,
     * and color sensor. Child classes can override this method to add additional hardware. In that case,
     * the first statement in the override method should be: super.createHardwareMap().
     */
    protected void createHardwareMap(){
        hardwareMap = new HardwareMap();
        PortConfig[] motorConfigs = new PortConfig[]{config.BL_MOTOR, config.FL_MOTOR, config.FR_MOTOR, config.BR_MOTOR};
        for (int i=0; i<4; i++){
            DcMotorControllerImpl controller = motorConfigs[i].HUB_TYPE == HubType.CONTROL_HUB ? motorController0 : motorController1;
            hardwareMap.put(motorConfigs[i].PORT_NAME, new DcMotorExImpl(config.MOTOR_TYPE, controller, motorConfigs[i].PORT_NUM));
        }
        hardwareMap.put(config.IMU_NAME, new BNO055IMUImpl(this, 10));
        hardwareMap.put(config.IMU_NAME, new BNO055IMUNew(this, 10));

        PortConfig[] encoderConfigs = new PortConfig[] {config.R_DEADWHEEL, config.L_DEADWHEEL, config.X_DEADWHEEL};
        for (int i=0; i<3; i++) {
            int driveMotorPort = -1;
            for (int j = 0; j < 4; j++) {
                if (motorConfigs[j].PORT_NUM == encoderConfigs[i].PORT_NUM) {
                    driveMotorPort = j;
                }
            }
            DcMotorControllerImpl controller = encoderConfigs[i].HUB_TYPE == HubType.CONTROL_HUB ? motorController0 : motorController1;
            if (driveMotorPort==-1) {
                hardwareMap.put(encoderConfigs[i].PORT_NAME, new DeadWheelEncoder(config.MOTOR_TYPE, controller, encoderConfigs[i].PORT_NUM));
            } else{
                redirect[i] = driveMotorPort;
            }
        }
    }

    /**
     * Update the position of the robot on the field, as well as the distance, BNO055IMU, and color sensors.
     *
     * Updating robot position involves:
     *     1) Get new position and orientation from the dyn4j physics engine.
     *     2) Using kinetic model, "preload" the chassis body with force and torque to be applied
     *        during the next update of the physics engine.
     *
     * @param millis milliseconds since the previous update
     */
    public synchronized void updateStateAndSensors(double millis){

        //Save old x, y, and headingRadians values for updating free wheel encoders later
        double xOld = x;
        double yOld = y;
        double headingOld = headingRadians;

        //Compute new pose and update various sensors
        /*
         * Get updated position and heading from the dyn4j body (chassisBody)
         */
        double xMeters = chassisBody.getTransform().getTranslationX();
        double yMeters = chassisBody.getTransform().getTranslationY();
        x = xMeters * VirtualField.PIXELS_PER_METER;
        y = yMeters * VirtualField.PIXELS_PER_METER;
        headingRadians = chassisBody.getTransform().getRotationAngle();

        // Compute new wheel speeds in pixel units per second

        double[] wSpd = new double[4];
        for (int i=0; i<4; i++){
            motors[i].update(millis);
            wSpd[i] = motors[i].getVelocity(AngleUnit.RADIANS) * config.GEAR_RATIO_WHEEL * wheelCircumference  / (2.0 * Math.PI);
            boolean mtRev = config.MOTOR_TYPE.REVERSED;
            boolean dirRev = motors[i].getDirection() == DcMotorSimple.Direction.REVERSE;
            if (
                    i<2 && (mtRev && dirRev || !mtRev && !dirRev) || i>=2 && (mtRev && !dirRev || !mtRev && dirRev)
            ) wSpd[i] = -wSpd[i];
        }

        /*
         * Based on wheel speeds, compute the target final robot velocity and angular speed, in the robot
         * coordinate system.
         */
        double[] robotTargetSpd = new double[]{0,0,0,0};
        for (int i=0; i<4; i++){
            for (int j=0; j<4; j++){
                robotTargetSpd[i] += tWR[i][j] * wSpd[j];
            }
        }

        robotTargetSpd[0] /= VirtualField.PIXELS_PER_METER;
        robotTargetSpd[1] /= VirtualField.PIXELS_PER_METER;

        /*
         * Compute an estimated final heading. This is used only to convert the target final robot velocity
         * from robot coordinate system to world coordinate system.
         */
        double t = millis / 1000.0;
        double estFinalHeading = headingRadians + 0.5 * (chassisBody.getAngularVelocity() + robotTargetSpd[2]) * t;
        double sinFinal = Math.sin(estFinalHeading);
        double cosFinal = Math.cos(estFinalHeading);

        /*
         * Convert target final robot velocity from robot coordinate system to world coordinate system.
         */
        Vector2 estFinalVelocity = new Vector2(
                robotTargetSpd[0]*cosFinal - robotTargetSpd[1]*sinFinal,
                robotTargetSpd[0]*sinFinal + robotTargetSpd[1]*cosFinal
        );

        /*
         * Compute the force and torque that would be required to achieve the target changes in robot velocity and
         * angular speed (F = ma,  tau = I*alpha)
         */
        Vector2 force = estFinalVelocity.difference(chassisBody.getLinearVelocity()).product(chassisBody.getMass().getMass()/t);
        double torque = (robotTargetSpd[2] - chassisBody.getAngularVelocity()) * chassisBody.getMass().getInertia()/t;

        /*
         * Convert the proposed force from world to robot coordinate system.
         */
        double sinHd = Math.sin(headingRadians);
        double cosHd = Math.cos(headingRadians);

        float fXR = (float)(force.x*cosHd + force.y*sinHd);
        float fYR = (float)(-force.x*sinHd + force.y*cosHd);

        // VectorF containing total force and torque required on bot to achieve the tentative position change,
        // in robot coordinate system
        VectorF totalForce = new VectorF(fXR, fYR, (float)torque, 0);

        // Required friction force from floor to achieve the tentative position change. For now, this will
        // be the same as totalForce, but may want to add offsets to collision forces
        VectorF frictionForces = new VectorF(totalForce.get(0), totalForce.get(1), totalForce.get(2), totalForce.get(3));

        // Determine the X-direction forces that would be required on each of the bot's four wheels to achieve
        // the total frictional force and torque predicted by the kinematic model. Note that the magnitude of
        // total force on each wheel is sqrt(2) times abs(x-direction force). (ROBOT coordinate system)

        VectorF wheel_X_Forces = M_ForceRobotToWheel.multiplied(frictionForces);

        //If any of the wheel forces exceeds the product of muStatic*mass*gravity, reduce the magnitude
        //of that force to muKinetic*mass*gravity, keeping the direction the same

        for (int i=0; i<4; i++){
            float f = wheel_X_Forces.get(i);
            if (Math.abs(f) > maxWheelXForce) {
                wheel_X_Forces.put(i, maxWheelXForce * Math.signum(f));
            }
        }

        //Based on the adjusted forces at each wheel, determine net frictional force and torque on the bot,
        //Force is in ROBOT COORDINATE system

        frictionForces = M_ForceWheelToRobot.multiplied(wheel_X_Forces);

        // Convert these adjusted friction forces to WORLD COORDINATES  and put into the original
        // force and torque variables
        force = new Vector2(frictionForces.get(0)*cosHd - frictionForces.get(1)*sinHd,
                frictionForces.get(0)*sinHd + frictionForces.get(1)*cosHd);
        torque = frictionForces.get(2);

        /*
         * Apply the adjusted frictional force and torque to the chassisBody
         *
         * Note:  We are only applying the frictional forces from the floor, NOT the collision forces. The
         *        collision forces will be applied automatically during the next update of the world by the
         *        dyn4j physics engine.
         */

        chassisBody.applyForce(force);
        chassisBody.applyTorque(torque);

        /*
         * Update the sensors
         */
        imu.updateHeadingRadians(headingRadians);
        imuNew.updateHeadingRadians(headingRadians);

        //For the deadwheel encoders, recalculate dXR and dYR to take into account the fact that the robot
        //may have run into the wall.
        double deltaX = x - xOld;
        double deltaY = y - yOld;
        double headingChange = AngleUtils.normalizeRadians(headingRadians - headingOld);
        double avgHeading = AngleUtils.normalizeRadians(headingOld + 0.5 * headingChange);
        double sin = Math.sin(avgHeading);
        double cos = Math.cos(avgHeading);

        double dxR = deltaX * cos + deltaY * sin;
        double dyR = -deltaX * sin + deltaY * cos;

        //Compute radians of rotation of each dead wheel encoder
        double rightEncoderRadians = (dyR + rightEncoderX * headingChange) / encoderWheelRadius;
        double leftEncoderRadians = -(dyR + leftEncoderX * headingChange) / encoderWheelRadius;
        double xEncoderRadians = -(dxR - xEncoderY * headingChange) / encoderWheelRadius;

        //Update positions of the dead wheel encoders
        rightEncoder.update(rightEncoderRadians, millis);
        leftEncoder.update(leftEncoderRadians, millis);
        xEncoder.update(xEncoderRadians, millis);

    }

    /**
     * Display the robot in the current orientation and position.
     */
    public synchronized void updateDisplay(){
        super.updateDisplay();
    }

    public void powerDownAndReset(){
        for (int i = 0; i < 4; i++) motors[i].stopAndReset();
        imu.close();

        chassisBody.setAngularVelocity(0);
        chassisBody.setLinearVelocity(0,0);

        rightEncoder.stopAndReset();
        leftEncoder.stopAndReset();
        xEncoder.stopAndReset();
    }

    /**
     *  Set up the chassisBody and add it to the dyn4j world. This method creates a Body and adds a BodyFixture
     *  containing a Rectangle. Add the chassis body to the world.
     *     The density value of 71.76 kg/m2 results in chassis mass of 15 kg.
     *     The "friction" of 0 refers to robot-game element and robot-wall friction (NOT robot-floor)
     *     The "restitution" of 0 refers to "bounce" when robot collides with wall and game elements
     *
     *     May want to change density, friction, and restitution to obtain desired behavior
     *
     *  The filter set on the chassisFixture indicates what other things the robot is capable of colliding with
     */
    public void setUpChassisBody(){
        chassisBody = new Body();
        chassisBody.setUserData(this);
        double botWidthMeters = botWidth / VirtualField.PIXELS_PER_METER;
        chassisFixture = chassisBody.addFixture(
                new org.dyn4j.geometry.Rectangle(botWidthMeters, botWidthMeters), 71.76, 0, 0);
        chassisRectangle = (org.dyn4j.geometry.Rectangle)chassisFixture.getShape();
        chassisFixture.setFilter(Filters.CHASSIS_FILTER);
        chassisBody.setMass(MassType.NORMAL);
        world.addBody(chassisBody);
    }
}
