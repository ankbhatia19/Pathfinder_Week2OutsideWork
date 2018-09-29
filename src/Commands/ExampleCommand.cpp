/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "ExampleCommand.h"
#include "ctre/Phoenix.h"
#include <pathfinder.h>



ExampleCommand::ExampleCommand() {

	TalonSRX* talon = new TalonSRX(0);

	talon->Set(ControlMode::PercentOutput, 0.5);
	talon->Set(ControlMode::Velocity, 1);
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(&Robot::chassis);
}

// Called just before this Command runs the first time
void ExampleCommand::Initialize()
{

}

// Called repeatedly when this Command is scheduled to run
void ExampleCommand::Execute()
{
		int POINT_LENGTH = 3;

		Waypoint points[POINT_LENGTH];

		Waypoint p1 = { -4, -1, d2r(45) };      // Waypoint @ x=-4, y=-1, exit angle=45 degrees
		Waypoint p2 = { -1, 2, 0 };             // Waypoint @ x=-1, y= 2, exit angle= 0 radians
		Waypoint p3 = {  2, 4, 0 };   	          // Waypoint @ x= 2, y= 4, exit angle= 0 radians
		points[0] = p1;
		points[1] = p2;
		points[2] = p3;

		TrajectoryCandidate candidate;

		// Prepare the Trajectory for Generation.
		//
		// Arguments:
		// Fit Function:        FIT_HERMITE_CUBIC or FIT_HERMITE_QUINTIC
		// Sample Count:        PATHFINDER_SAMPLES_HIGH (100 000)
		//                      PATHFINDER_SAMPLES_LOW  (10 000)
		//                      PATHFINDER_SAMPLES_FAST (1 000)
		// Time Step:           0.001 Seconds
		// Max Velocity:        15 m/s
		// Max Acceleration:    10 m/s/s
		// Max Jerk:            60 m/s/s/s
		pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_HIGH, 0.001, 15.0, 10.0, 60.0, &candidate);

		int length = candidate.length;

		// Array of Segments (the trajectory points) to store the trajectory in

		Segment* traj = malloc(length * sizeof(Segment));

		// Generate the trajectory
		pathfinder_generate(&candidate, traj);

		Segment leftTrajectory[length];
		Segment rightTrajectory[length];

		// The distance between the left and right sides of the wheelbase is 0.6m
		double wheelbase_width = 0.6;

		// Generate the Left and Right trajectories of the wheelbase using the
		// originally generated trajectory
		pathfinder_modify_tank(traj, length, leftTrajectory, rightTrajectory, wheelbase_width);
		EncoderFollower follower = malloc(sizeof(EncoderFollower));
		follower.last_error = 0; follower.segment = 0; follower.finished = 0;



		EncoderConfig config = { 0, 1000, 6 * 3.14,      // Position, Ticks per Rev, Wheel Circumference
		                         1.0, 0.0, 0.0, 1.0 / 1.0, 0.0};


	// find this part out
	double l = pathfinder_follow_encoder(leftConfig, &leftFollower, leftTrajectory, trajectory_length, l_encoder_value);
	double r = pathfinder_follow_encoder(rightConfig, &rightFollower, rightTrajectory, trajectory_length, r_encoder_value);
}

// Make this return true when this Command no longer needs to run execute()
bool ExampleCommand::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void ExampleCommand::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ExampleCommand::Interrupted() {}
