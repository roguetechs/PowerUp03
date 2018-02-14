/*-------------------------------------------------------------------*/
/* Copyright (c) 2018 TEAM 3953. All Rights Reserved.               */
/*-------------------------------------------------------------------*/

// -----------------------------------------------------------
// DRIVE BASE – ALL Speed Controllers on CAN Bus
// -----------------------------------------------------------
// TalonSRX ID 1 = Right Main Drive
// TalonSRX ID 2 = Right Follower Main Drive
// TalonSRX ID 3 = Left Main Drive
// TalonSRX ID 4 = Left Follower Main Drive

#include "WPILib.h"
#include <iostream>
#include <memory>
#include <string>
#include <stdio.h>
#include "ctre/Phoenix.h"          // required for TalonSRX / VictorSRX

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

class Robot : public frc::IterativeRobot{
	public:

	TalonSRX  *leftMotorMaster;
	TalonSRX  *leftMotorSlave;
	TalonSRX  *rightMotorMaster;
	TalonSRX  *rightMotorSlave;
	VictorSPX *clawGrab;
	VictorSPX *clawRotate;
	VictorSPX *cubeInOut;
	VictorSPX *liftMech;

	Joystick  *driver;
	Joystick  *coDriver;

		  void RobotInit(){
			// see variable declarations in private section (bottom)
			// assign the Speed Controllers to the CAN BUS ID’s
			m_motorRight = new TalonSRX(1);       // #1 - #2 Follows #1
			m_motorRightSlave = new TalonSRX(2);
			m_motorLeft = new TalonSRX(3);         // #3 - #4 Follows #3
			m_motorLeftSlave = new TalonSRX(4);

			clawGrab = new VictorSPX(5);
			clawRotate = new VictorSPX(6);
			cubeInOut = new VictorSPX(7);
			liftMech = new VictorSPX(8);

			// assign the controller object to Joystick 0 in driver station
			driver = new Joystick(0);
			coDriver = new Joystick(1);

			// setup the follower CANTalons for the drive base
			m_motorRightSlave->Set(ControlMode::Follower, 1); // follow #1
			m_motorLeftSlave->Set(ControlMode::Follower, 3); // follow #3
		  }

		  void AutonomousInit() override{
		  }

		  void AutonomousPeriodic(){
		  }


		  void TeleopInit(){
		  }

		  void TeleopPeriodic(){

			double motor_left;
			double motor_right;

			// ---------------------------------------------------------------
			// get controller axis positions
			// ---------------------------------------------------------------
			motor_left = driver->GetY();
			motor_right = driver ->GetRawAxis(4) * -1.0; // inverted
			// ---------------------------------------------------------------

			// ---------------------------------------------------------------
			// set speed controller output to the drive base motors
			// ---------------------------------------------------------------
			m_motorLeft->Set(ControlMode::PercentOutput, motor_left);
			m_motorRight->Set(ControlMode::PercentOutput, motor_right);
			// ---------------------------------------------------------------
		}

		void TestPeriodic(){
		}

		void DisabledPeriodic(){
		}



	private:
		  // declare the variables for speed controllers & joystick
		  TalonSRX *m_motorLeft;            // #1 - #2 Follows #1
		  TalonSRX *m_motorLeftSlave;
		  TalonSRX *m_motorRight;            // #4 - #3 Follows #3
		  TalonSRX *m_motorRightSlave;

		  Joystick *driver;           // XBOX Controller
};

START_ROBOT_CLASS(Robot)
