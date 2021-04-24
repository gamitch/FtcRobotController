// ***********************************************************************
// RevTankTeleOp
// ***********************************************************************
// The tele-op mode for RevTank operations

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// ***********************************************************************
// Definitions from Qualcomm code for OpMode recognition
// ***********************************************************************
@TeleOp(name="RevTank: 1-TeleOp 1.0", group="RevTank")
//@Disable

public class RevTankTeleOp extends RevTankCore {
    // scale the power used for drive motors
    private double  drivePowerFactor;

    // joystick checking
    private double  nextMoveTolerance;

    // ***********************************************************************
    // RevTankTeleOp
    // ***********************************************************************
    // Constructs the class.
    // The system calls this member when the class is instantiated.
    public RevTankTeleOp()
    {
        // Initialize base classes.
        // All via self-construction.

        // Initialize class members.
        // All via self-construction.
    }

    // ***********************************************************************
    // Init
    // ***********************************************************************
    // Performs any actions that are necessary when the OpMode is enabled.
    // The system calls this member once when the OpMode is enabled.
    @Override public void init ()
    {
        revTankDebug(500, "RevTankTeleOp::init", "START");

        // Run initialization of other parts of the class
        // Note that the class will connect to all of our motors and servos
        super.init();

        // Set the default drive power
        drivePowerFactor = 0.8;

        // Joystick move tolerance
        nextMoveTolerance = 0.4;

        // read autonomous values liek alliance color
        revTankReadAutoSettings();

        revTankDebug(500, "RevTankTeleOp::init", "DONE");
    }


    // ***********************************************************************
    // start
    // ***********************************************************************
    // Do first actions when the start command is given.
    // Called once when the OpMode is started.
    @Override
    public void start()
    {
        revTankDebug(500, "RevTankTeleOp::start", "START");

        // Call the super/base class start method.
        super.start();

        // Slowly shift the wheel power
        useGradualWheelChange = Boolean.FALSE;

        revTankDebug(500, "RevTankTeleOp::start", "DONE");
    }


    // ***********************************************************************
    // loop
    // ***********************************************************************
    // State machine for autonomous robot control
    // Called continuously while OpMode is running
    @Override
    public void loop() {
        double  driveLeft;
        double  driveRight;

        revTankDebug(500, "RevTankTeleOp::loop", "START");

        // use left joystick Y value for left speed (invert, since Y is negative going up)
        if (( gamepad1.left_stick_y > nextMoveTolerance ) || ( gamepad1.left_stick_y < -nextMoveTolerance )) {
            driveLeft = -gamepad1.left_stick_y * drivePowerFactor;
        } else {
            driveLeft = 0;
        }

        // use right joystick Y value for left speed (invert, since Y is negative going up)
        if (( gamepad1.right_stick_y > nextMoveTolerance ) || ( gamepad1.right_stick_y < -nextMoveTolerance )) {
            driveRight = -gamepad1.right_stick_y * drivePowerFactor;
        } else {
            driveRight = 0;
        }

        //slow down
        if (gamepad1.right_trigger > minTriggerMove) {
            driveLeft *= 0.45;
            driveRight *= 0.45;
        }
        //speed up
        if (gamepad1.left_trigger > minTriggerMove) {
            driveLeft *= 1.5;
            driveRight *= 1.5;
        }

        revTankDebug(1000, "RevTankTeleOp::loop", "move read");

        // Move the robot,
        moveRobot(driveLeft, driveRight);



        // Any loop background updates happen now....
        loopEndReporting();

        revTankDebug(500, "RevTankTeleOp::loop", "DONE");
    }


    // ***********************************************************************
    // stop
    // ***********************************************************************
    // Performs any actions that are necessary when the OpMode is disabled.
    // The system calls this member once when the OpMode is disabled.
    @Override
    public void stop() {
        revTankDebug( 500, "RevTankTeleOp::stop", "START" );

        // Call the super/base class stop method
        super.stop();

        revTankDebug(500, "RevTankTeleOp::stop", "DONE" );
    }

}
