// ***********************************************************************
// RevTankCore
// ***********************************************************************
//
// EXAMPLE code for a tank bot with autonomous using IMU for driving
// -- Developed for 2019-2020 season kickoff by Vince Westin, FTA Lead for FTC in GA
// -- Send comments or questions to vincewestin@gmail.com
//
// Extends the OpMode class to provide a single hardware access point for our robot.
//
// It also gives a common way to do things like reading and saving autonomous settings
//
// All operation control for the RevTank robot starts here.
//
// We have included all of the motor and servo definitions, so that they do not have to be
// kept up in all of the individual files.
//
// We also included the functions for moving the various key motors and servos or any other
// basic robot operations. It simplifies the code in our other program files.


package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.text.DecimalFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

// ***********************************************************************
// Definitions from Qualcomm code for OpMode recognition
//
// Not used in our core class extension
// ***********************************************************************
@TeleOp(name="RevTank: 99-core", group="RevTank")
@Disabled

// ***********************************************************************
// RevTankCore
// ***********************************************************************
// Class definitions

public class RevTankCore extends OpMode {
    //
    // Hardware mapping of devices
    //
    // Wheel motors for the Mecanum wheels
    DcMotor motorLeftFront;
    DcMotor motorLeftRear;
    DcMotor motorRightFront;
    DcMotor motorRightRear;
    DcMotor motorLift;

    // target power levels for the wheel motors
    double  targetLeft;
    double  targetRight;
    // assume not using gradual wheel change
    boolean useGradualWheelChange = Boolean.FALSE;
    // private copies of current wheel power
    private double currentLeft = 0.0;
    private double currentRight = 0.0;

    // Servos for pushing the buttons
    Servo   servoGrab;

    DigitalChannelController digitalInterface;

    // *** Sensors ***
    VoltageSensor   batteryVoltSensor;
    BNO055IMU ourIMU;
    // saved current (adjusted) orietnation/heading
    double curHeading;
    // for logging
    private String orientationLog;


    // IMU calibration file
    String imuCalibration = "RevIMUCalibration.json";


    // ***** //
    // Values for autonomous activities
    // ***** //
    /* Are we done reading settings? */
    Boolean settingsDone;
    // Are we red?
    Boolean amIRed = Boolean.TRUE;
    // Delay at start
    int autoDelay = 0;
    static final int delayMAX = 12000;
    // start from hang?
    Boolean autoHangStart = Boolean.FALSE;


    //
    // Controller handling values
    //

    // When reading the controllers, ignore small bumps
    static double minJoystickMove = 0.2;
    static double minTriggerMove = 0.3;


    //
    // File and message information
    //

    // Run time data
    private String startDate;
    private ElapsedTime fullTime = new ElapsedTime();
    private ElapsedTime runTime = new ElapsedTime();

    // Number output format
    DecimalFormat revTankNumberFormat;

    // Level of debug data to show on driver station
    int     debugLevel = 499;

    // distance is in cm, we have ~144 encoder ticks per rotation and a 6 inch wheel diameter
    // encoder ticks per cm of lateral distance
    double wheelcm2encoder = ( 144 / ( 2.54 * ( 6 * Math.PI )));

    // Files where we store settings
    private String revTankFileAutoSettings;


    // ***********************************************************************
    // RevTankCore
    // ***********************************************************************
    // Constructs the class.
    // The system calls this member when the class is instantiated.
    public RevTankCore() {
        // Initialize base classes.
        // All via self-construction.

        // Initialize class members.
        // All via self-construction.
    }


    // ***********************************************************************
    // init
    // ***********************************************************************
    // Performs any actions that are necessary when the OpMode is enabled.
    // The system calls this member once when the OpMode is enabled.
    // For this overall class, we build out all the defaults for robot control
    @Override
    public void init() {

        revTankDebug(500, "RevTankCore::init", "START");

        // Default format for our numbers
        revTankNumberFormat = new DecimalFormat("0.00");

        // Record the starting time for this OpMode
        startDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date());
        fullTime.reset();

        revTankDebug(500, "RevTankCore::init", "DATE done");

        // Set the names for the setting files
        //  Note that the directory matches the other configs for the robot
        revTankFileAutoSettings = AppUtil.FIRST_FOLDER + "RevTankAuto.dat";

        //
        // Use the hardwareMap to associate class members to hardware ports.
        //
        /*
         * *********** NOTE *********
         * The names of these devices are loaded from the XML file on the phone.
         * We can use "adb push" to add files onto the phone, rather than typing them in.
         * The files on the phone live in /sdcard/FIRST/...
         */

        // Motors for the wheels
        // --- run without encoder still gives encoder values, just does not move based on targets
        motorLeftFront = hardwareMap.dcMotor.get("LF_motor");
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        revTankDebugDevice(500, "Left Front Motor", motorLeftFront);
        motorLeftRear = hardwareMap.dcMotor.get("LR_motor");
        motorLeftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        revTankDebugDevice(500, "Left Rear Motor", motorLeftRear);
        motorRightFront = hardwareMap.dcMotor.get("RF_motor");
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        revTankDebugDevice(500, "Right Front Motor", motorRightFront);
        motorRightRear = hardwareMap.dcMotor.get("RR_motor");
        motorRightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        revTankDebugDevice(500, "Right Rear Motor", motorRightRear);
  //      motorLift = hardwareMap.dcMotor.get("Lift");
  //      motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  //      revTankDebugDevice(500,"Spinner", motorLift);

        // Some motors need to move in opposite directions by default
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorLeftRear.setDirection(DcMotor.Direction.REVERSE);

        revTankDebug(500, "RevTankCore::init", "MOTORS done");

        // Servos
       // servoGrab = hardwareMap.servo.get("Grab");
        //revTankDebugDevice(500, "Grab Servo", servoGrab);
        // limit range of grab servo movement
        //servoGrab.scaleRange(0.0,0.4);
        // Move opposite of left
//        servoLeftPush.setDirection(Servo.Direction.REVERSE);

        //revTankDebug(500, "LeviCore::init", "SERVOS detected");

        //
        // Sensors
        //

        // No hardware connected yet...

        // battery power
        // We found a very useful link to the battery voltage here:
        //    https://www.reddit.com/r/FTC/comments/3odx26/is_it_possible_to_get_the_battery_voltage/
        batteryVoltSensor = hardwareMap.voltageSensor.get("Expansion Hub 173");
        revTankDebugDevice(500,"Battery Voltage Sensor", batteryVoltSensor);

        // Connect to the IMU in the primary hub
        // Rev has a built-in IMU for relative position information. The swerve drive uses the IMU.
        ourIMU = hardwareMap.get(BNO055IMU.class, "imu");

        // Set up the parameters with which we will use our IMU.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        parameters.calibrationDataFile = imuCalibration; // from calibration sample
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        // activate the parameters
        ourIMU.initialize(parameters);

        revTankDebug(500,"Inertial management unit", "connected");


        revTankDebug(500, "RevTankCore::init", "SENSORS connected");

        revTankDebug( 500, "RevTankCore::init", "DONE");
    }


    // ***********************************************************************
    // start
    // ***********************************************************************
    // Performs any actions that are necessary when the OpMode is enabled.
    // The system calls this member once when the OpMode is enabled.
    @Override
    public void start() {
        // Only actions that are common to all Op-Modes (i.e. both automatic and
        // manual) should be implemented here.
        //
        // This method is designed to be overridden.

        revTankDebug(500, "RevTankCore::start", "START");

        super.start();

        // Now that the robot has really started, note this as the real start time
        resetStartTime();
        runTime.reset();

        // no current motor movement
        currentLeft = currentRight = 0.0;

        // Record current orientation/heading
        checkOrientation();

        revTankDebug(500, "RevTankCore::start", "DONE");
    }


    // ***********************************************************************
    // loop
    // ***********************************************************************
    // Performs any actions that are necessary while the OpMode is running.
    // The system calls this member repeatedly while the OpMode is running.
    @Override
    public void loop() {
        // Only actions that are common to all OpModes (i.e. both auto and\
        // manual) should be implemented here.
        //
        // This method is designed to be overridden.

        // Core loop only needs to be logged for very high debug levels
        revTankDebug(5000, "RevTankCore::loop", "loop run");

        loopEndReporting();
    }


    // ***********************************************************************
    // stop
    // ***********************************************************************
    // Performs any actions that are necessary when the OpMode is disabled.
    // The system calls this member once when the OpMode is disabled.
    @Override
    public void stop() {
        // Nothing needs to be done for this OpMode.
        revTankDebug(500, "RevTankCore::stop", "Stop run");

        super.stop();
    }


    // ***********************************************************************
    // loopEndReporting
    // ***********************************************************************
    // Perform any background updates.
    // Report on core robot status
    void loopEndReporting() {
        // Loop reporting only needs to be logged for very high debug levels
        revTankDebug( 5000, "RevTankCore::loopEndReporting", "loopEndReporting run");

        // Note run time
        revTankLog("1 Start", "Core started at " + startDate);
        revTankLog("2 Status", "running for " + runTime.toString());

        // Current controler values
        revTankLog("  CTL 1", controllerTelemetry(gamepad1));
        revTankLog("  CTL 2", controllerTelemetry(gamepad2));

        // Last orientation
        revTankLog( "Orient", revTankNumberFormat.format(curHeading));
    }


    // ***********************************************************************
    // moveRobot
    // ***********************************************************************
    // Cause the robot drive wheels to turn
    // Based on the compass definitions of CR_*
    // The Mecanum wheels allow us to move in each possible direction....
    //
    // The NE/NW/SE/SW moves have a touch of forward/backward to help the robot glide
    // Nice in idea, but the robot mostly went forward. Even divided by 50.
    void moveRobot(double speedLeft, double speedRight) {

        double  absPower;

        revTankLog("MoveRobot", "Left: " + revTankNumberFormat.format(speedLeft)
                + " Right: " + revTankNumberFormat.format(speedRight));

        // be sure the power level is not too high
        absPower = Math.abs(speedLeft);
        if (absPower > 1.0) {
            speedLeft /= absPower;
        }
        absPower = Math.abs(speedRight);
        if (absPower > 1.0) {
            speedRight /= absPower;
        }

        targetLeft = speedLeft;
        targetRight = speedRight;

        // now set the power of each motor
        updateWheelPower();

        // note the diagnostics on the display
        revTankLog("Motors",
                "Left: " + revTankNumberFormat.format(targetLeft)
                        + "  Right: " + revTankNumberFormat.format(targetRight));
    }


    // ***********************************************************************
    // updateWheelPower
    // ***********************************************************************
    // Set the actual wheel motion power
    private void updateWheelPower() {

        // if we are using gradual change, shift to the next step
        if (useGradualWheelChange) {
            currentLeft = gradualNext(currentLeft, targetLeft);
            currentRight = gradualNext(currentRight, targetRight);

            // or we just use the target
        } else {
            currentLeft = targetLeft;
            currentRight = targetRight;
        }

        // now set the power of each motor
        motorLeftFront.setPower(currentLeft);
        motorLeftRear.setPower(currentLeft);
        motorRightFront.setPower(currentRight);
        motorRightRear.setPower(currentRight);
    }


    // ***********************************************************************
    // gradualNext
    // ***********************************************************************
    // Set the actual wheel motion power
    private double gradualNext( double current, double target ) {
        double difference;
        double nextShift;

        difference = Math.abs( current - target );

        // if very close, just make them match
        if ( difference < 0.05 ) {
            return( target );

            // if getting close, shift by 0.05
        } else if ( difference < 0.3 ) {
            nextShift = 0.05;

            // else shift by 0.1
        } else {
            nextShift = 0.1;
        }

        // move slower if needed
        if ( current > target ) {
            nextShift = -nextShift;
        }

        return( current + nextShift );
    }


    // ***********************************************************************
    // checkOrientation - gather the current orientation data
    // ***********************************************************************
    void checkOrientation() {
        // and the angles from that IMU
        Orientation angles;
        Acceleration gravAngles;

        // read the orientation of the robot
        angles = ourIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // and save the heading
        curHeading = - ( angles.firstAngle );
    }


    // ***********************************************************************
    // controllerTelemetry
    // ***********************************************************************
    // String of all the current controller values
    private String controllerTelemetry(Gamepad myPad) {
        String leftTrigger;
        String leftBumper;
        String rightTrigger;
        String rightBumper;
        String leftRightPush;
        String abxy;
        String answer;

        if (myPad.left_trigger > minTriggerMove) {
            leftTrigger = " LT: " + myPad.left_trigger;
        } else {
            leftTrigger = "";
        }
        if (myPad.left_bumper) {
            leftBumper = " LB";
        } else {
            leftBumper = "";
        }
        if (myPad.right_trigger > minTriggerMove) {
            rightTrigger = " RT: " + myPad.right_trigger;
        } else {
            rightTrigger = "";
        }
        if (myPad.right_bumper) {
            rightBumper = " RB";
        } else {
            rightBumper = "";
        }
        if (myPad.left_stick_button) {
            leftRightPush = " LSP";
        } else {
            leftRightPush = "";
        }
        if (myPad.right_stick_button) {
            leftRightPush += " RSP";
        }
        if (myPad.a) {
            abxy = " A";
        } else {
            abxy = "";
        }
        if (myPad.b) {
            abxy += " B";
        }
        if (myPad.x) {
            abxy += " X";
        }
        if (myPad.y) {
            abxy += " Y";
        }

        answer = " LX: " + revTankNumberFormat.format(myPad.left_stick_x)
                + " LY: " + revTankNumberFormat.format(myPad.left_stick_y)
                + " RX: " + revTankNumberFormat.format(myPad.right_stick_x)
                + " RY: " + revTankNumberFormat.format(myPad.right_stick_y)
                + leftRightPush
                + leftTrigger + leftBumper
                + rightTrigger + rightBumper
                + abxy;

        return answer;
    }


    // ***********************************************************************
    // revTankReadAutoSettings
    // ***********************************************************************
    // Read file of autonomous goal settings
    int revTankReadAutoSettings() {
        FileInputStream myFile;
        int myValue;                // Becomes a single BYTE that is read....

        try {
            myFile = new FileInputStream(revTankFileAutoSettings);

            myValue = myFile.read();
            if ( myValue == 1 ) {
                amIRed = Boolean.TRUE;
            } else {
                amIRed = Boolean.FALSE;
            }

            autoDelay = myFile.read();
            // If the delay is invalid, then we ignore it....
            if ( autoDelay > delayMAX ) {
                autoDelay = delayMAX;
            }

            myFile.close();

        } catch (Exception e) {
            // nothing much to do....
            return -1;
        }

        return 0;
    }

    // ***********************************************************************
    // revTankWriteAutoSettings
    // ***********************************************************************
    // Write file of autonomous goal settings
    int revTankWriteAutoSettings() {
        FileOutputStream myFile;
        int myValue;                // Becomes a single BYTE to write....

        try {
            myFile = new FileOutputStream(revTankFileAutoSettings, false);

            if ( amIRed ) {
                myValue = 1;
            } else {
                myValue = 0;
            }
            myFile.write(myValue);

            myFile.write(autoDelay);

            // Close out the file
            myValue = '\n';
            myFile.write(myValue);

            myFile.close();

        } catch (Exception e) {
            // nothing much to do....
            return -1;
        }

        return 0;
    }

    // ***********************************************************************
    // revTankSleep
    // ***********************************************************************
    // Go to sleep and wait, time is in milliseconds
    void revTankSleep(long millis ) {
        double  startTime;
        double  now;
        long    delta;

        revTankDebug(500, "RevTankCore::revTankSleep", "START, requested time is " + millis + "ms");

        startTime = getRuntime();
        do {
            now = getRuntime();
            delta = (long)((now - startTime) * 1000);
        } while (delta < millis);

        revTankDebug(500, "RevTankCore::revTankSleep", "DONE, elapsed time is " + delta + "ms");
    }


    // ***********************************************************************
    // revTankDebugDevice
    // ***********************************************************************
    // Debugging messages for device map
    private void revTankDebugDevice(int myLevel, String myName, HardwareDevice myDevice) {
        String  message;

        message = myName + "== name: " + myDevice.getDeviceName()
                + ", connect: " + myDevice.getConnectionInfo()
                + ", version: " + myDevice.getVersion();

        // Show the debug as telemetry if set for that level of debug
        if ( debugLevel > myLevel ) {
            telemetry.addData("**DEBUG DEVICE**", message);
        }

        // Add debug data to the log...
        RobotLog.i("**DEBUG DEVICE** " + message);
    }


    // ***********************************************************************
    // revTankDebug
    // ***********************************************************************
    // Debugging messages
    void revTankDebug(int myLevel, String myName, String myMessage ) {

        // Show the debug as telemetry if set for that level of debug
        if ( debugLevel > myLevel ) {
            telemetry.addData( "**DEBUG**: " + myName, myMessage );
        }

        // Add debug data to the log...
        // -- for very high levels of debug item, ONLY add if debugging at that level
        if ((1000 > myLevel) || (debugLevel > myLevel)) {
            RobotLog.i("**DEBUG** == " + myName + ": " + myMessage);
        }
    }


    // ***********************************************************************
    // revTankLog
    // ***********************************************************************
    // Log messages that are always shown
    void revTankLog(String myName, String myMessage ) {

        // Show the message on the driver display
        telemetry.addData( myName, myMessage);

        // Add debug data to the log...w
        RobotLog.i( "LOG == " + myName + ": " + myMessage );
    }
}
