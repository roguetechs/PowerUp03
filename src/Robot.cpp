/*-------------------------------------------------------------------*/
/* Copyright (c) 2018 TEAM 3953. All Rights Reserved.               */
/*-------------------------------------------------------------------*/

// -----------------------------------------------------------
// DRIVE BASE – ALL Speed Controllers on CAN Bus
// -----------------------------------------------------------
//  TalonSRX ID 1 = Right Main Drive
//  TalonSRX ID 2 = Right Follower Main Drive
//  TalonSRX ID 3 = Left Main Drive
//  TalonSRX ID 4 = Left Follower Main Drive
// VictorSPX ID 5 = Claw Open And Close
// VictorSPX ID 6 = Rotate The Claw
// VictorSPX ID 7 = Power Cube Intake Mechanism
// VictorSPX ID 8 = Lift Mechanism Master Motor
// VictorSPX ID 9 = Lift Mechanism Slave Motor

// ---------------------------------------------------------------
// Include libraries to use in the code
// ---------------------------------------------------------------
#include <memory>
#include <string>
#include <stdio.h>
#include <Timer.h>
#include "WPILib.h"
#include <iostream>
#include <RobotDrive.h>
#include "ctre/Phoenix.h"          // required for TalonSRX / VictorSRX
#include <RobotController.h>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

class Robot : public frc::IterativeRobot{
	public:

		// ---------------------------------------------------------------
		// Define Motors
		// ---------------------------------------------------------------

		// Drive Motors
		TalonSRX  *leftMotorMaster;
		TalonSRX  *leftMotorSlave;
		TalonSRX  *rightMotorMaster;
		TalonSRX  *rightMotorSlave;
		// Manipulator Motors
		VictorSPX *clawGrab;
		VictorSPX *clawRotate;
		VictorSPX *cubeInOut;
		VictorSPX *cubeInOut_2;
		VictorSPX *liftMech;
		VictorSPX *liftMechSlave;

		// ---------------------------------------------------------------
		// Define Controllers
		// ---------------------------------------------------------------
		Joystick  *driver;
		Joystick  *coDriver;

		// ---------------------------------------------------------------
		// Define Code Variables
		// ---------------------------------------------------------------

		// Timer - Match Time
		int timer;

		// Button 2 - Lift Mechanism Up
		bool B2;
		// Button 1 - Lift Mechanism Down
		bool B1;
		// Button 5 - Power Cube Intake (wheels)
		bool B5;
		bool B5state;
		// Button 4 - Power Cube Output (wheels)
		bool B4;
		bool B4state;
		// Intake and Output state variable
		int inOutState = 0;
		// POV - Rotate Claw
		int POV;
		// POV Up variable
		bool POVup;
		bool POVup2;
		// POV Down variable
		bool POVdown;
		bool POVdown2;
		// Claw Rotate state variable
		int POVstate = 0;
		// Button 7 - Master Toggle for driver
		bool B7_1;
		// Button 7 - Master Toggle for coDriver
		bool B7_2;

	//bool Button2State = false;

		void RobotInit(){
			// see variable declarations in private section (bottom)
			// assign the Speed Controllers to the CAN BUS ID’s

			// ---------------------------------------------------------------
			// Drive Motor
			// ---------------------------------------------------------------

			m_motorRight = new TalonSRX(1);       // #1 - #2 Follows #1
			m_motorRightSlave = new TalonSRX(2);
			m_motorLeft = new TalonSRX(3);         // #3 - #4 Follows #3
			m_motorLeftSlave = new TalonSRX(4);

			clawGrab = new VictorSPX(5);
			clawRotate = new VictorSPX(6);
			cubeInOut = new VictorSPX(7);
			cubeInOut_2 = new VictorSPX(8);
			liftMech = new VictorSPX(9);
			liftMechSlave = new VictorSPX(10);

			// assign the controller object to Joystick 0 in driver station
			driver = new Joystick(0);
			coDriver = new Joystick(1);

			// setup the follower CANTalons for the drive base
			m_motorRightSlave->Set(ControlMode::Follower, 1); // follow #1
			m_motorLeftSlave->Set(ControlMode::Follower, 3); // follow #3
			//cubeInOut_2->Set(ControlMode::Follower, 7); // follow #7
			liftMechSlave->Set(ControlMode::Follower, 9); // follow #9
		}

		void AutonomousInit() override{
		}

		void AutonomousPeriodic(){
		}


		void TeleopInit(){
		}

		void TeleopPeriodic(){
			// ---------------------------------------------------------------
			// Set speed variables
			// Define buttons and controllers
			// ---------------------------------------------------------------

			// driver controller | variable for left and right speeds
			double motor_left;
			double motor_right;
			// B2 | coDriver controller | variable for lift up speed
			B2 = coDriver->GetRawButton(2);
			double lift_Up;
			// B3 | coDriver controller | variable for lift down speed
			B1 = coDriver->GetRawButton(1);
			double lift_Down;
			// B5 | coDriver controller | variable for power cube input speed
			B5 = coDriver->GetRawButton(5);
			double input_Speed;
			// B4 | coDriver controller | variable for power cube output speed
			B4 = coDriver->GetRawButton(4);
			double output_Speed;
			// POV Up | coDriver controller | variable for rotating claw up speed
			POV = coDriver->GetPOV();
			double rotate_Up;
			// POV Down | coDriver controller | variable for rotating claw down down
			double rotate_Down;
			// B7 | Both Controllers
			B7_1 = driver->GetRawButton(7);
			B7_2 = coDriver->GetRawButton(7);
			// Timer | Match Time | Used to time Master Toggle
			timer = Timer::GetMatchTime();

			// ---------------------------------------------------------------
			// Drive the robot code
			// ---------------------------------------------------------------

			// Get controller axis positions
			motor_left = driver->GetRawAxis(1);
			motor_right = driver->GetRawAxis(5);

			// set speed controller output to the drive base motors
			m_motorLeft->Set(ControlMode::PercentOutput, motor_left);
			m_motorRight->Set(ControlMode::PercentOutput, motor_right);

			// ---------------------------------------------------------------
			// Create a press and hold system for the lift mechanism to go up and down
			// Up = Button 2 | Down = Button 1
			// ---------------------------------------------------------------

			// Set the lift speed for going up and down
			lift_Up = 0.5;
			lift_Down = -0.5;

			if(B2){
				liftMech->Set(ControlMode::PercentOutput, lift_Up);
			}
			// Set the lift mechanism to go down
			else if(B1){
				liftMech->Set(ControlMode::PercentOutput, lift_Down);
			}
			// Set the lift mechanism to stop when no button is pressed
			else {
				liftMech->Set(ControlMode::PercentOutput, 0.0);
			}

			// ---------------------------------------------------------------
			// Create a toggle for the system for the claw to open and close
			// Toggle = Button 0
			// ---------------------------------------------------------------



			// ---------------------------------------------------------------
			// Create a toggle to intake and output power cubes (Spin the Wheels)
			// Cube in = Button 5 | Cube out = Button 4
			// ---------------------------------------------------------------

			// Define the speed of the input and output wheels
			input_Speed = 0.5;
			output_Speed = -0.5;

			// Control the toggle for the intake and output mechanism
			switch(inOutState){
				// Neutral state (not moving)
				case 0:
					cubeInOut->Set(ControlMode::PercentOutput, 0.0);
					cubeInOut_2->Set(ControlMode::PercentOutput, 0.0);
					// If button 2 is pressed then set to case 1
					if((!B5state) && (B5)){
						inOutState ++;
					}
					// If button 1 is pressed then set to case -1
					if((!B4state) && (B4)){
						inOutState --;
					}
				break;
				// Activate the wheels for intake
				case 1:
					cubeInOut->Set(ControlMode::PercentOutput, input_Speed);
					cubeInOut_2->Set(ControlMode::PercentOutput, input_Speed * -1);
					// If button 2 is pressed then set to case 0
					if((!B5state) && (B5)){
						inOutState --;
					}
					// If button 1 is pressed then set to case -1
					if((!B4state) && (B4)){
						inOutState -= 2;
					}
				break;
				// Activate the wheels for output
				case -1:
					cubeInOut->Set(ControlMode::PercentOutput, output_Speed);
					cubeInOut_2->Set(ControlMode::PercentOutput, output_Speed * -1);
					// If button 1 is pressed then set to case 0
					if((!B4state) && (B4)){
						inOutState ++;
					}
					// If button 2 is pressed then set to case -1
					if((B5state) && (B5)){
						inOutState += 2;
					}
				break;
			}
			// Set the state values to equal the corresponding buttons
			B5state = B5;
			B4state = B4;

			// ---------------------------------------------------------------
			// Rotate claw up and down on a toggle system
			// Rotate up = POV Up | Rotate down = POV Down
			// ---------------------------------------------------------------

			// Define the speed the claw will rotate at
			rotate_Up = 0.25;
			rotate_Down = -0.25;

			// Read the POV and decide if the POV is up or down
			if(POV == 0){
				POVup = true;
			}
			else {
				POVup = false;
			}

			if(POV == 180){
				POVdown = true;
			}
			else {
				POVdown = false;
			}
			// Control the toggle for rotating the claw
			switch(POVstate){
				// Neutral state (not moving)
				case 0:
					clawRotate->Set(ControlMode::PercentOutput, 0.0);
					// If POV is up then rotate the claw up
					if((POVup) && (!POVup2)){
						POVstate ++;
					}
					// If POV is down then rotate the claw down
					if((POVdown) && (!POVdown2)){
						POVstate --;
					}
				break;
				// Rotate the claw up
				case 1:
					clawRotate->Set(ControlMode::PercentOutput, rotate_Up);
					// If POV is up then set to case 0
					if((POVup) && (!POVup2)){
						POVstate --;
					}
					// If POV is down then set to case -1
					if((POVdown) && (!POVdown2)){
						POVstate -= 2;
					}
				break;
				// Rotate the claw down
				case -1:
					clawRotate->Set(ControlMode::PercentOutput, rotate_Down);
					// If POV is down then set to case 0
					if((POVdown) && (!POVdown2)){
						POVstate ++;
					}
					// If POV is up then set to case -1
					if((POVup) && (!POVup2)){
						POVstate += 2;
					}
				break;
			}
			// Define second variables to be their first variables
			POVup2 = POVup;
			POVdown2 = POVdown;

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

		//Joystick *driver;           // XBOX Controller
};

START_ROBOT_CLASS(Robot)
