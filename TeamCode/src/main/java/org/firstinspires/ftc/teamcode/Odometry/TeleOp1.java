/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Odometry;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name= "TeleOp V.1")
public class TeleOp1 extends LinearOpMode {

    OdometryGlobalCoordinatePosition globalPositionUpdate;
    /* Declare OpMode members. */
    Hardware robot           = new Hardware();   // Use hardware map
    @Override
    public void runOpMode() {
        double FrontLeft;
        double FrontRight;
        double RearLeft;
        double RearRight;
        double r;
        double robotAngle;
        double rightX;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        globalPositionUpdate=new OdometryGlobalCoordinatePosition(robot.verticalLeft,robot.verticalRight,robot.horizontal,305.5774907364,75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();
        double initialX=globalPositionUpdate.returnXCoordinate();
        double initialY=globalPositionUpdate.returnYCoordinate();
        //send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        //wait for the game to start (driver presses PLAY)
        waitForStart();

        //run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //do calculations to convert stick position to motor power
            r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            rightX = gamepad1.right_stick_x;
            FrontLeft = r * Math.cos(robotAngle) - rightX;
            FrontRight = r * Math.sin(robotAngle) + rightX;
            RearLeft = r * Math.sin(robotAngle) - rightX;
            RearRight = r * Math.cos(robotAngle) + rightX;

            //set values to motor powers
            robot.FL.setPower(-FrontLeft);
            robot.FR.setPower(FrontRight);
            robot.RR.setPower(RearRight);
            robot.RL.setPower(RearLeft);


            if(gamepad1.a)
            {
                setRobotOrientation(90, 0.25);
            }
            if(gamepad1.b)
            {
                setRobotPosition(initialX, initialY, 0.25,globalPositionUpdate.returnOrientation());
            }
            if(gamepad1.x)
            {
                setRobotOrientation(270, 0.25);
                setRobotOrientation(90, 0.25);
                setRobotOrientation(270, 0.25);
                setRobotOrientation(90, 0.25);
                setRobotOrientation(270, 0.25);
                setRobotOrientation(90, 0.25);
                setRobotOrientation(270, 0.25);
                setRobotOrientation(90, 0.25);
                setRobotOrientation(270, 0.25);
                setRobotOrientation(90, 0.25);
            }
        }
    }
    public void setRobotOrientation(double targetAngle, double speed)
    {
        if(targetAngle > 180)
        {
            speed = -speed;
        }
        while (globalPositionUpdate.returnOrientation() < targetAngle)
        {
            robot.FR.setPower(speed);
            robot.RR.setPower(speed);
            robot.FL.setPower(-speed);
            robot.RL.setPower(-speed);
        }
    }
    public void setRobotPosition(double targetX, double targetY, double speed, double targetOrient)
    {
        double deltaX = globalPositionUpdate.returnXCoordinate()- targetX;
        double deltaY = globalPositionUpdate.returnYCoordinate()- targetY;
        double initOrientation = globalPositionUpdate.returnOrientation();
        double deltaO = initOrientation - targetOrient;
        double magnitude = Math.hypot(deltaX,deltaY);
        double droppedMag = .8*magnitude;





        while(Math.hypot(globalPositionUpdate.returnXCoordinate()-targetX,globalPositionUpdate.returnYCoordinate()-targetY)>0.1)//0.0 can be changed to increase chance of completion
        {

            double distanceX = targetX - globalPositionUpdate.returnXCoordinate();
            double distanceY = targetY - globalPositionUpdate.returnYCoordinate();
            double angle = Math. atan2(distanceY, distanceX);
            double FrontLeft =-speed*Math.cos(angle+initOrientation);
            double FrontRight = speed*Math.cos(angle+initOrientation);
            double RearLeft = speed*Math.cos(angle+initOrientation);
            double RearRight = speed*Math.cos(angle+initOrientation);
            robot.FL.setPower(FrontLeft);
            robot.FR.setPower(FrontRight);
            robot.RR.setPower(RearRight);
            robot.RL.setPower(RearLeft);
        }
    }
}

