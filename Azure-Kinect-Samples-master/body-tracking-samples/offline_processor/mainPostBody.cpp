// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <iostream>
#include <fstream>
#include <iomanip>
#include <math.h>
#include <string>
#include <iostream>
#include <sstream>
#include <chrono>
#include <thread>
#include <stdio.h>
#include <malloc.h>
#include <k4a/k4a.h>
#include <k4arecord/playback.h>
#include <k4abt.h>

#include <nlohmann/json.hpp>

#include <k4a/BodyTrackingHelpers.h>
#include <k4a/Utilities.h>

using namespace std;
using namespace nlohmann;

typedef struct
{
	char* filename;
	k4a_playback_t handle;
	k4a_record_configuration_t record_config;
	k4a_capture_t capture;
} recording_t;

//DU Modification -- Eddy Rogers
//Struct to contain the minimum and maximum acceptable x, y, and z values
struct bounding_box {
	k4a_float3_t minimum;
	k4a_float3_t maximum;
};

//DU Modification -- Eddy Rogers
//Struct to contain useful information about the playbacks specified by the command line input
struct playback_info {
	recording_t *files;
	k4a_calibration_t* calibrations;
	uint64_t master_recording_length;
	size_t file_count;
};

static uint64_t first_capture_timestamp(k4a_capture_t capture)
{
	// DU MOD
	uint64_t timestamp = k4a_image_get_device_timestamp_usec(k4a_capture_get_depth_image(capture));
	k4a_image_release(k4a_capture_get_depth_image(capture));
	return timestamp;
	// DU MOD
}

static void create_xy_table(const k4a_calibration_t* calibration, k4a_image_t xy_table)
{
	k4a_float2_t* table_data = (k4a_float2_t*)(void*)k4a_image_get_buffer(xy_table);

	int width = calibration->depth_camera_calibration.resolution_width;
	int height = calibration->depth_camera_calibration.resolution_height;

	k4a_float2_t p;
	k4a_float3_t ray;
	int valid;

	for (int y = 0, idx = 0; y < height; y++)
	{
		p.xy.y = (float)y;
		for (int x = 0; x < width; x++, idx++)
		{
			p.xy.x = (float)x;

			k4a_calibration_2d_to_3d(
				calibration, &p, 1.f, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray, &valid);

			if (valid)
			{
				table_data[idx].xy.x = ray.xyz.x;
				table_data[idx].xy.y = ray.xyz.y;
			}
			else
			{
				table_data[idx].xy.x = nanf("");
				table_data[idx].xy.y = nanf("");
			}
		}
	}
}



//DU Modification -- Eddy Rogers
//Creates a bounding box struct corresponding to the maximum and minimum detected joint positions
bounding_box create_bounding_box(k4a_float3_t* jointPositions)
{
	bounding_box bounds;
	bounds.maximum = jointPositions[0];
	bounds.minimum = jointPositions[0];

	for (int i = 0; i < 32; i++)
	{
		k4a_float3_t currentPoint = jointPositions[i];
		if (currentPoint.xyz.x >= bounds.maximum.xyz.x)
		{
			bounds.maximum.xyz.x = currentPoint.xyz.x;
		}
		if (currentPoint.xyz.y >= bounds.maximum.xyz.y)
		{
			bounds.maximum.xyz.y = currentPoint.xyz.y;
		}
		if (currentPoint.xyz.z >= bounds.maximum.xyz.z)
		{
			bounds.maximum.xyz.z = currentPoint.xyz.z;
		}
		if (currentPoint.xyz.x < bounds.minimum.xyz.x)
		{
			bounds.minimum.xyz.x = currentPoint.xyz.x;
		}
		if (currentPoint.xyz.y < bounds.minimum.xyz.y)
		{
			bounds.minimum.xyz.y = currentPoint.xyz.y;
		}
		if (currentPoint.xyz.z < bounds.minimum.xyz.z)
		{
			bounds.minimum.xyz.z = currentPoint.xyz.z;
		}
	}
	bounds.maximum.xyz.x += 100; bounds.maximum.xyz.y += 100; bounds.maximum.xyz.z += 100;
	bounds.minimum.xyz.x -= 100; bounds.minimum.xyz.y -= 100; bounds.minimum.xyz.z -= 100;
	return bounds;
}

//Returns whether an x, y, and z are acceptable given a bounding box
bool within_bounds(float x, float y, float z, bounding_box bounds) {
	return ((x >= bounds.minimum.xyz.x && x <= bounds.maximum.xyz.x)
		&& (y >= bounds.minimum.xyz.y && y <= bounds.maximum.xyz.y)
		&& (z >= bounds.minimum.xyz.z && z <= bounds.maximum.xyz.z));
}
//End DU Modification

//DU Modification -- Eddy Rogers
//Added a list of joints as an argument, and checks if each point falls within the bounds given by the joint locations
static void generate_point_cloud(const k4a_image_t depth_image, const k4a_image_t ir_image,
	const k4a_image_t xy_table,
	k4a_image_t point_cloud,
	int* point_count,
	k4a_float3_t* jointPositions,
	bool bounding)
{
	int width = k4a_image_get_width_pixels(depth_image);
	int height = k4a_image_get_height_pixels(depth_image);

	uint16_t* depth_data = (uint16_t*)(void*)k4a_image_get_buffer(depth_image);
	uint16_t* ir_data = (uint16_t*)(void*)k4a_image_get_buffer(ir_image);
	k4a_float2_t* xy_table_data = (k4a_float2_t*)(void*)k4a_image_get_buffer(xy_table);
	k4a_float3_t* point_cloud_data = (k4a_float3_t*)(void*)k4a_image_get_buffer(point_cloud);

	bounding_box bounds = create_bounding_box(jointPositions);

	*point_count = 0;
	for (int i = 0; i < width * height; i++)
	{
		//DU Modification: Check if the point is within the acceptable range of points close to a joint
		if (depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y))
			//&& (ir_data[i] != 0 && ir_data[i] <= 400))
		{
			if (bounding) {
				//std::cout << "Bounding...\n";
				// * (float)depth_data[i]
				if (within_bounds(xy_table_data[i].xy.x * (float)depth_data[i], xy_table_data[i].xy.y * (float)depth_data[i], (float)depth_data[i], bounds)) {
					point_cloud_data[i].xyz.x = xy_table_data[i].xy.x * (float)depth_data[i];
					point_cloud_data[i].xyz.y = xy_table_data[i].xy.y * (float)depth_data[i];
					point_cloud_data[i].xyz.z = (float)depth_data[i];
					(*point_count)++;
					//std::cout << "Point Written...\n";
				}
			}
			else {
				point_cloud_data[i].xyz.x = xy_table_data[i].xy.x * (float)depth_data[i];
				point_cloud_data[i].xyz.y = xy_table_data[i].xy.y * (float)depth_data[i];
				point_cloud_data[i].xyz.z = (float)depth_data[i];
				(*point_count)++;
			}
		}
		else
		{
			point_cloud_data[i].xyz.x = nanf("");
			point_cloud_data[i].xyz.y = nanf("");
			point_cloud_data[i].xyz.z = nanf("");
		}
	}
}

static void write_point_cloud(const char* file_name, const k4a_image_t point_cloud, int point_count)
{
	int width = k4a_image_get_width_pixels(point_cloud);
	int height = k4a_image_get_height_pixels(point_cloud);

	k4a_float3_t* point_cloud_data = (k4a_float3_t*)(void*)k4a_image_get_buffer(point_cloud);

	// save to the ply file
	std::ofstream ofs(file_name); // text mode first
	ofs << "ply" << std::endl;
	ofs << "format ascii 1.0" << std::endl;
	ofs << "element vertex"
		<< " " << point_count << std::endl;
	ofs << "property float x" << std::endl;
	ofs << "property float y" << std::endl;
	ofs << "property float z" << std::endl;
	ofs << "end_header" << std::endl;
	ofs.close();

	std::stringstream ss;
	for (int i = 0; i < width * height; i++)
	{
		if (isnan(point_cloud_data[i].xyz.x) || isnan(point_cloud_data[i].xyz.y) || isnan(point_cloud_data[i].xyz.z))
		{
			continue;
		}

		ss << (float)point_cloud_data[i].xyz.x << " " << (float)point_cloud_data[i].xyz.y << " "
			<< (float)point_cloud_data[i].xyz.z << std::endl;
	}

	std::ofstream ofs_text(file_name, std::ios::out | std::ios::app);
	ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}

//DU Modification -- Eddy Rogers
//Retrieve input from the user, and return an int representing the state the program should run in
//The user is asked to specify whether they want the JSON object, point clouds, or both.
//0 -- JSON Only
//1 -- Point Cloud Only
//2 -- Both
int get_state_user_input()
{
	std::string userInput;

	//Output instructions to the user
	std::cout << "Please specify whether you want the program to output JSON only, point clouds only, or both JSON and point clouds.\n";
	std::cout << "To do this, type \"JSON\", \"POINT\", or \"BOTH\" (non case sensitive)\n";

	//Get user input
	std::cin >> userInput;

	//Transform the input into lowercase
	int length = std::string(userInput).size();
	for (int i = 0; i < length; i++)
	{
		userInput[i] = std::tolower(userInput[i]);
	}

	//Compare the input string
	if (userInput.compare("json") == 0) return 0;
	if (userInput.compare("point") == 0) return 1;
	if (userInput.compare("both") == 0) return 2;

	//If incorrectly formatted, just try again
	else {
		std::cout << "Input formatted incorrectly. Please try again\n";
		return get_state_user_input();
	}
}

//DU Modification -- Eddy Rogers
//Retrieve input from the user, and return a bool representing whether the user wants to use a bounding box.
bool get_bounding_input()
{
	std::string userInput;

	//Output instructions to the user
	std::cout << "Please specify whether or not you want the program to use joint locations to filter out background points.\n";
	std::cout << "To do this, type \"Y\" or \"N\" (non case sensitive)\n";

	//Get user input
	std::cin >> userInput;

	//Transform the input into lowercase
	int length = std::string(userInput).size();
	for (int i = 0; i < length; i++)
	{
		userInput[i] = std::tolower(userInput[i]);
	}

	//Compare the input string
	if (userInput.compare("y") == 0) return true;
	if (userInput.compare("n") == 0) return false;

	//If incorrectly formatted, just try again
	else {
		std::cout << "Input formatted incorrectly. Please try again\n";
		return get_state_user_input();
	}
}

bool get_master_subordinate_input()
{
	std::string userInput;

	//Output instructions to the user
	std::cout << "Would you like the program to process MKV files recorded in Master/Subordinate mode? Y/N\n";

	//Get user input
	std::cin >> userInput;

	//Transform the input into lowercase
	int length = std::string(userInput).size();
	for (int i = 0; i < length; i++)
	{
		userInput[i] = std::tolower(userInput[i]);
	}

	//Compare the input string
	if (userInput.compare("y") == 0) return true;
	if (userInput.compare("n") == 0) return false;

	//If incorrectly formatted, just try again
	else {
		std::cout << "Input formatted incorrectly. Please try again\n";
		return get_master_subordinate_input();
	}
}

//DU Modification -- Eddy Rogers
//Added a list of xyz coordinates to be used in point cloud bounding algorithm, and an int to serve as a weight for how likely the data is to be accurate.
bool predict_joints(json& frames_json, int frame_count, k4abt_tracker_t tracker, k4a_capture_t capture_handle, 
						k4a_float3_t* jointPositions)
{
	k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, capture_handle, K4A_WAIT_INFINITE);
	if (queue_capture_result != K4A_WAIT_RESULT_SUCCEEDED)
	{
		cerr << "Error! Adding capture to tracker process queue failed!" << endl;
		return false;
	}

	k4abt_frame_t body_frame = nullptr;
	k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
	if (pop_frame_result != K4A_WAIT_RESULT_SUCCEEDED)
	{
		cerr << "Error! Popping body tracking result failed!" << endl;
		return false;
	}

	size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
	uint64_t timestamp = k4abt_frame_get_device_timestamp_usec(body_frame);

	json frame_result_json;
	frame_result_json["timestamp_usec"] = timestamp;
	frame_result_json["frame_id"] = frame_count;
	frame_result_json["num_bodies"] = num_bodies;
	frame_result_json["bodies"] = json::array();
	for (size_t i = 0; i < num_bodies; i++)
	{
		k4abt_skeleton_t skeleton;
		VERIFY(k4abt_frame_get_body_skeleton(body_frame, i, &skeleton), "Get body from body frame failed!");
		json body_result_json;
		int body_id = k4abt_frame_get_body_id(body_frame, i);
		body_result_json["body_id"] = body_id;

		int total_confidence = 0;

		for (int j = 0; j < (int)K4ABT_JOINT_COUNT; j++)
		{

			body_result_json["joint_positions"].push_back({ skeleton.joints[j].position.xyz.x,
																skeleton.joints[j].position.xyz.y,
																skeleton.joints[j].position.xyz.z });

			//Insert the joint poistion into the array
			jointPositions[j] = skeleton.joints[j].position;

			body_result_json["joint_orientations"].push_back({ skeleton.joints[j].orientation.wxyz.w,
																skeleton.joints[j].orientation.wxyz.x,
																skeleton.joints[j].orientation.wxyz.y,
																skeleton.joints[j].orientation.wxyz.z });

			total_confidence += skeleton.joints[j].confidence_level;

			//Pushes the confidence level of each joint back on the JSON group containing the quaternions and the positions
			body_result_json["joint_confidence"].push_back({ skeleton.joints[j].confidence_level });
		}

		body_result_json["total_confidence"] = total_confidence;

		frame_result_json["bodies"].push_back(body_result_json);
	}
	frames_json.push_back(frame_result_json);
	k4abt_frame_release(body_frame);

	return true;
}

bool check_depth_image_exists(k4a_capture_t capture)
{
	k4a_image_t depth = k4a_capture_get_depth_image(capture);
	if (depth != nullptr)
	{
		k4a_image_release(depth);
		return true;
	}
	else
	{
		return false;
	}
}

//DU Modification -- Eddy Rogers
//Modified the function arguments to take a list of recordings, calibrations, the length of the master recording, and how many files to process
//Modified the function to run through each recording provided and produce a JSON object for each of them
bool process_mkv_offline_json(playback_info playback, bool bounding) {
	
	//Array of booleans to hold whether or not each file was processed
	bool* success = new bool[playback.file_count];
	for (int i = 0; i < playback.file_count; i++)
	{
		success[i] = true;
	}

	//For each given file
	for (int i = 2; i < 3; i++)
	{
		//Open the recording
		k4a_playback_t playback_handle = nullptr;
		k4a_result_t result = k4a_playback_open(playback.files[i].filename, &playback_handle);
		if (result != K4A_RESULT_SUCCEEDED)
		{
			cerr << "Cannot open recording at " << playback.files[i].filename << endl;
			return false;
		}
		
		//Determine the calibration of the recording
		k4a_calibration_t calibration;
		result = k4a_playback_get_calibration(playback_handle, &calibration);
		if (result != K4A_RESULT_SUCCEEDED)
		{
			cerr << "Failed to get calibration" << endl;
			return false;
		}

		//Create a body tracker with a default configuration
		k4abt_tracker_t tracker = NULL;
		k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
		result = k4abt_tracker_create(&calibration, tracker_config, &tracker);
		if (result != K4A_RESULT_SUCCEEDED)
		{
			cerr << "Body tracker initialization failed!" << endl;
			return false;
		}

		//Create a JSON object to be output
		json json_output;
		json_output["k4abt_sdk_version"] = K4ABT_VERSION_STR;
		json_output["source_file"] = playback.files[i].filename;

		// Store all joint names to the json
		json_output["joint_names"] = json::array();
		for (int i = 0; i < (int)K4ABT_JOINT_COUNT; i++)
		{
			json_output["joint_names"].push_back(g_jointNames.find((k4abt_joint_id_t)i)->second);
		}

		// Store all bone linkings to the json
		json_output["bone_list"] = json::array();
		for (int i = 0; i < (int)g_boneList.size(); i++)
		{
			json_output["bone_list"].push_back({ g_jointNames.find(g_boneList[i].first)->second,
												 g_jointNames.find(g_boneList[i].second)->second });
		}

		cout << "Tracking " << playback.files[i].filename << endl;

		int frame_count = 0;
		json frames_json = json::array();
		
		//For each frame
		while (true)
		{
			//Get the next capture
			k4a_capture_t capture_handle = nullptr;
			k4a_stream_result_t stream_result = k4a_playback_get_next_capture(playback_handle, &capture_handle);
			if (stream_result == K4A_STREAM_RESULT_EOF)
			{
				break;
			}

			cout << "frame " << frame_count << '\r';

			if (stream_result == K4A_STREAM_RESULT_SUCCEEDED)
			{
				// Only try to predict joints when capture contains depth image
				if (check_depth_image_exists(capture_handle))
				{
					//DU Modification: Store the location of each joint for this frame
					k4a_float3_t* joints = new k4a_float3_t[32];

					//Predict the joint locations, saving them onto the JSON array
					success[i] = predict_joints(frames_json, frame_count, tracker, capture_handle, joints);

					if (!success[i])
					{
						cerr << "Predict joints failed for clip at frame " << frame_count << endl;
						break;
					}

					k4a_capture_release(capture_handle);
				}
			}
			else
			{
				success[i] = false;
				cerr << "Stream error for clip at frame " << frame_count << endl;
				break;
			}

			frame_count++;
		}

		if (success[i])
		{
			json_output["frames"] = frames_json;
			cout << endl << "DONE " << endl;

			cout << "Total read " << frame_count << " frames" << endl;

			std::string json_filename = std::string(playback.files[i].filename);

			//Remove the ".mkv" from the json, and add ".json" extension
			json_filename.resize(json_filename.size() - 4);
			json_filename.append(".json");

			//Create and open the output file, and ensure that it is open before writing
			std::cout << "Creating Output File...";
			std::ofstream output_file;
			output_file.open(json_filename.c_str());
			if (output_file.is_open() == false)
			{
				cerr << "Unable to open JSON file to write." << std::endl;
				break;
			}

			//Write to the output file
			output_file << std::setw(4) << json_output << std::endl;
			cout << "Results saved in " << json_filename.c_str() << std::endl;
		}

		//Close the playback, and destroy the tracker so we can open new ones for the next iteration
		k4a_playback_close(playback_handle);
		k4abt_tracker_destroy(tracker);
	}
	
	//Return value for the function
	bool total_success = true;
	//Run through the array of success values - if any are false, we return false;
	for (int i = 0; i < playback.file_count; i++)
	{
		total_success &= success[i];
	}
	return total_success;
}
//End DU Modification

//DU Modification -- Eddy Rogers
//Modified the function arguments to take a list of recordings, calibrations, the length of the master recording, and how many files to process
//Modified the program to process one frame from each camera and apply bounds to the found body or bodies
bool process_mkv_offline_point(playback_info playback, bool bounding)
{
	//Array of booleans to hold whether or not each file was processed
	bool* success = new bool[playback.file_count];
	for (int i = 0; i < playback.file_count; i++)
	{
		success[i] = true;
	}

	//For each input file
	for (int i = 0; i < playback.file_count; i++)
	{
		//Open the file for playback
		k4a_playback_t playback_handle = nullptr;
		k4a_result_t result = k4a_playback_open(playback.files[i].filename, &playback_handle);
		if (result != K4A_RESULT_SUCCEEDED)
		{
			cerr << "Cannot open recording at " << playback.files[i].filename << endl;
			return false;
		}

		//Get the calibration for the recording
		k4a_calibration_t calibration;
		result = k4a_playback_get_calibration(playback_handle, &calibration);
		if (result != K4A_RESULT_SUCCEEDED)
		{
			cerr << "Failed to get calibration" << endl;
			return false;
		}

		//Create a body tracker with the default configuration
		k4abt_tracker_t tracker = NULL;
		k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
		result = k4abt_tracker_create(&calibration, tracker_config, &tracker);
		if (result != K4A_RESULT_SUCCEEDED)
		{
			cerr << "Body tracker initialization failed!" << endl;
			return false;
		}

		//Create a JSON object to be output
		json json_output;
		json_output["k4abt_sdk_version"] = K4ABT_VERSION_STR;
		json_output["source_file"] = playback.files[i].filename;

		// Store all joint names to the json
		json_output["joint_names"] = json::array();
		for (int i = 0; i < (int)K4ABT_JOINT_COUNT; i++)
		{
			json_output["joint_names"].push_back(g_jointNames.find((k4abt_joint_id_t)i)->second);
		}

		// Store all bone linkings to the json
		json_output["bone_list"] = json::array();
		for (int i = 0; i < (int)g_boneList.size(); i++)
		{
			json_output["bone_list"].push_back({ g_jointNames.find(g_boneList[i].first)->second,
												 g_jointNames.find(g_boneList[i].second)->second });
		}

		cout << "Tracking " << playback.files[i].filename << endl;

		int frame_count = 0;
		json frames_json = json::array();

		//For each frame
		while (true)
		{
			//Get the next capture
			k4a_capture_t capture_handle = nullptr;
			k4a_stream_result_t stream_result = k4a_playback_get_next_capture(playback_handle, &capture_handle);
			if (stream_result == K4A_STREAM_RESULT_EOF)
			{
				break;
			}

			cout << "frame " << frame_count << '\r';

			if (stream_result == K4A_STREAM_RESULT_SUCCEEDED)
			{
				// Only try to predict joints when capture contains depth image
				if (check_depth_image_exists(capture_handle))
				{
					//DU Modification: Store the location of each joint for this frame
					k4a_float3_t* joints = new k4a_float3_t[32];

					success[i] = predict_joints(frames_json, frame_count, tracker, capture_handle, joints);
					
					//Empty images to be populated and used to create the point cloud
					k4a_image_t xy_table = NULL;
					k4a_image_t point_cloud = NULL;
					k4a_image_t depthImage = NULL;
					k4a_image_t irImage = NULL;

					//Set the XY table
					k4a_result_t result1 = k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
						playback.calibrations[i].depth_camera_calibration.resolution_width,
						playback.calibrations[i].depth_camera_calibration.resolution_height,
						playback.calibrations[i].depth_camera_calibration.resolution_width * (int)sizeof(k4a_float2_t),
						&xy_table);

					create_xy_table(&playback.calibrations[i], xy_table);

					//Set the point cloud
					k4a_result_t result2 = k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
						playback.calibrations[i].depth_camera_calibration.resolution_width,
						playback.calibrations[i].depth_camera_calibration.resolution_height,
						playback.calibrations[i].depth_camera_calibration.resolution_width * (int)sizeof(k4a_float3_t),
						&point_cloud);

					//Get the depth image
					depthImage = k4a_capture_get_depth_image(capture_handle);
					irImage = k4a_capture_get_ir_image(capture_handle);

					//Generate the point cloud
					int point_count = 0;

					generate_point_cloud(depthImage, irImage, xy_table, point_cloud, &point_count, joints, bounding);

					//Save it to a file

					std::ostringstream outputFileName;


					//Take the .mkv off of the file
					std::string* fileName = new std::string(playback.files[i].filename);
					fileName->resize(fileName->size() - 4);

					outputFileName << fileName->c_str() << '.' << frame_count << ".ply";

					write_point_cloud(outputFileName.str().c_str(), point_cloud, point_count);

					k4a_capture_release(capture_handle);
				}
			}
			else
			{
				success[i] = false;
				cerr << "Stream error for clip at frame " << frame_count << endl;
				break;
			}
			frame_count++;
		}
		//Close the playback, and destroy the tracker so we can open new ones for the next iteration
		k4a_playback_close(playback_handle);
		k4abt_tracker_destroy(tracker);
	}

	//Return value for the function
	bool total_success = true;
	//Run through the array of success values - if any are false, we return false;
	for (int i = 0; i < playback.file_count; i++)
	{
		total_success &= success[i];
	}
	return total_success;
}
//End DU Modification

//DU Modification -- Eddy Rogers
//Modified the function arguments to take a list of recordings, calibrations, the length of the master recording, and how many files to process
//Modified the program to process one frame from each camera and apply bounds to the found body or bodies
bool process_mkv_offline_both(playback_info playback, bool bounding)
{
	//Array of booleans to hold whether or not each file was processed
	bool* success = new bool[playback.file_count];
	for (int i = 0; i < playback.file_count; i++)
	{
		success[i] = true;
	}

	//For each input file
	for (int i = 2; i < playback.file_count - 1; i++)
	{
		//Open the file
		k4a_playback_t playback_handle = nullptr;
		k4a_result_t result = k4a_playback_open(playback.files[i].filename, &playback_handle);
		if (result != K4A_RESULT_SUCCEEDED)
		{
			cerr << "Cannot open recording at " << playback.files[i].filename << endl;
			return false;
		}

		//Get the calibration associated with the recording
		k4a_calibration_t calibration;
		result = k4a_playback_get_calibration(playback_handle, &calibration);
		if (result != K4A_RESULT_SUCCEEDED)
		{
			cerr << "Failed to get calibration" << endl;
			return false;
		}

		//Create a body tracker specific to the recording
		k4abt_tracker_t tracker = NULL;
		k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
		result = k4abt_tracker_create(&calibration, tracker_config, &tracker);
		if (result != K4A_RESULT_SUCCEEDED)
		{
			cerr << "Body tracker initialization failed!" << endl;
			return false;
		}

		//Create a JSON file to be output
		json json_output;
		json_output["k4abt_sdk_version"] = K4ABT_VERSION_STR;
		json_output["source_file"] = playback.files[i].filename;

		// Store all joint names to the json
		json_output["joint_names"] = json::array();
		for (int i = 0; i < (int)K4ABT_JOINT_COUNT; i++)
		{
			json_output["joint_names"].push_back(g_jointNames.find((k4abt_joint_id_t)i)->second);
		}

		// Store all bone linkings to the json
		json_output["bone_list"] = json::array();
		for (int i = 0; i < (int)g_boneList.size(); i++)
		{
			json_output["bone_list"].push_back({ g_jointNames.find(g_boneList[i].first)->second,
												 g_jointNames.find(g_boneList[i].second)->second });
		}

		cout << "Tracking " << playback.files[i].filename << endl;

		int frame_count = 0;
		json frames_json = json::array();

		//For each frame
		while (true)
		{
			//Get the next capture
			k4a_capture_t capture_handle = nullptr;
			k4a_stream_result_t stream_result = k4a_playback_get_next_capture(playback_handle, &capture_handle);
			if (stream_result == K4A_STREAM_RESULT_EOF)
			{
				break;
			}

			cout << "frame " << frame_count << '\r';

			if (stream_result == K4A_STREAM_RESULT_SUCCEEDED)
			{
				// Only try to predict joints when capture contains depth image
				if (check_depth_image_exists(capture_handle))
				{
					//DU Modification: Store the location of each joint for this frame
					k4a_float3_t* joints = new k4a_float3_t[32];

					success[i] = predict_joints(frames_json, frame_count, tracker, capture_handle, joints);

					//Empty images to be used in the generation of the point cloud
					k4a_image_t xy_table = NULL;
					k4a_image_t point_cloud = NULL;
					k4a_image_t depthImage = NULL;
					k4a_image_t irImage = NULL;


					//Set the XY table
					k4a_result_t result1 = k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
						playback.calibrations[i].depth_camera_calibration.resolution_width,
						playback.calibrations[i].depth_camera_calibration.resolution_height,
						playback.calibrations[i].depth_camera_calibration.resolution_width * (int)sizeof(k4a_float2_t),
						&xy_table);

					create_xy_table(&playback.calibrations[i], xy_table);

					//Set the point cloud
					k4a_result_t result2 = k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
						playback.calibrations[i].depth_camera_calibration.resolution_width,
						playback.calibrations[i].depth_camera_calibration.resolution_height,
						playback.calibrations[i].depth_camera_calibration.resolution_width * (int)sizeof(k4a_float3_t),
						&point_cloud);

					depthImage = k4a_capture_get_depth_image(capture_handle);
					irImage = k4a_capture_get_ir_image(capture_handle);

					//Generate the point cloud
					int point_count = 0;

					

					generate_point_cloud(depthImage, irImage, xy_table, point_cloud, &point_count, joints, bounding);

					std::ostringstream outputFileName;

					//Take the .mkv off of the file
					std::string* fileName = new std::string(playback.files[i].filename);
					fileName->resize(fileName->size() - 4);

					outputFileName << fileName->c_str() << '.' << frame_count << ".ply";

					write_point_cloud(outputFileName.str().c_str(), point_cloud, point_count);

					k4a_capture_release(capture_handle);
				}
			}
			else
			{
				success[i] = false;
				cerr << "Stream error for clip at frame " << frame_count << endl;
				break;
			}
			frame_count++;
		}
		//Close the playback, and destroy the tracker so we can open new ones for the next iteration
		k4a_playback_close(playback_handle);
		k4abt_tracker_destroy(tracker);

		if (success[i])
		{
			//Write the JSON object to a file
			json_output["frames"] = frames_json;
			cout << endl << "DONE " << endl;

			cout << "Total read " << frame_count << " frames" << endl;

			std::string json_filename = std::string(playback.files[i].filename);

			//Remove the ".mkv" from the json, and add ".json" extension
			json_filename.resize(json_filename.size() - 4);
			json_filename.append(".json");

			//Create and open the output file, and ensure that it is open before writing
			std::cout << "Creating Output File...";
			std::ofstream output_file;
			output_file.open(json_filename.c_str());
			if (output_file.is_open() == false)
			{
				cerr << "Unable to open JSON file to write." << std::endl;
				break;
			}

			//Write to the output file
			output_file << std::setw(4) << json_output << std::endl;
			cout << "Results saved in " << json_filename.c_str() << std::endl;
		}
	}

	//Return value for the function
	bool total_success = true;
	//Run through the array of success values - if any are false, we return false;
	for (int i = 0; i < playback.file_count; i++)
	{
		total_success &= success[i];
	}
	return total_success;
}
//End DU Modification

//DU Modification -- Eddy Rogers
//Gets useful information from the provided files from the users (to be used in JSON and point cloud processing) and stores it
	//in the given playback_info object
bool get_playback_info(int argc, char** argv, playback_info* playback, bool master_subordinate)
{
	//Amount of input files is all arguments except for argv[0]
	playback->file_count = (size_t)(argc - 1);
	bool master_found = false;
	k4a_result_t result = K4A_RESULT_SUCCEEDED;

	// Allocate memory to store the state of N recordings.
	playback->files = (recording_t*)malloc(sizeof(recording_t) * playback->file_count);
	playback->calibrations = (k4a_calibration_t*)malloc(sizeof(k4a_calibration_t) * playback->file_count);
	if (playback->files == NULL)
	{
		printf("Failed to allocate memory for playback (%zu bytes)\n", sizeof(recording_t) * playback->file_count);
		return 1;
	}
	memset(playback->files, 0, sizeof(recording_t) * playback->file_count);
	memset(playback->calibrations, 0, sizeof(k4a_calibration_t) * playback->file_count);

	// Open each recording file and validate they were recorded in master/subordinate mode.
	for (size_t i = 0; i < playback->file_count; i++)
	{
		//Eddy Rogers -- Is this line a security concern?
		//The names of each file is found consecutively in argv
		playback->files[i].filename = argv[i + 1];

		result = k4a_playback_open(playback->files[i].filename, &playback->files[i].handle);

		if (result != K4A_RESULT_SUCCEEDED)
		{
			printf("Failed to open file: %s\n", playback->files[i].filename);
			break;
		}

		result = k4a_playback_get_record_configuration(playback->files[i].handle, &playback->files[i].record_config);
		if (result != K4A_RESULT_SUCCEEDED)
		{
			printf("Failed to get record configuration for file: %s\n", playback->files[i].filename);
			break;
		}

		if (master_subordinate == true)
		{
			//Confirm that the mkv files were recorded in subordinate/master mode
			if (playback->files[i].record_config.wired_sync_mode == K4A_WIRED_SYNC_MODE_MASTER)
			{
				playback->master_recording_length = k4a_playback_get_recording_length_usec(playback->files[i].handle);
				printf("Opened master recording file: %s\n", playback->files[i].filename);
				if (master_found)
				{
					printf("ERROR: Multiple master recordings listed!\n");
					result = K4A_RESULT_FAILED;
					break;
				}
				else
				{
					//Found the master
					master_found = true;
				}
			}
			else if (playback->files[i].record_config.wired_sync_mode == K4A_WIRED_SYNC_MODE_SUBORDINATE)
			{
				printf("Opened subordinate recording file: %s\n", playback->files[i].filename);
			}
			else
			{
				printf("ERROR: Recording file was not recorded in master/sub mode: %s\n", playback->files[i].filename);
				result = K4A_RESULT_FAILED;
				break;
			}
		}

		// Start DU MOD
		k4a_result_t calibrationResult = k4a_playback_get_calibration(playback->files[i].handle, &playback->calibrations[i]);
		if (calibrationResult != K4A_RESULT_SUCCEEDED) {
			printf("Error getting calibration for: %d\n", i);
			return 1;
		}
		// End DU MOD

		// Read the first capture of each recording into memory.
		k4a_stream_result_t stream_result = k4a_playback_get_next_capture(playback->files[i].handle, &playback->files[i].capture);
		if (stream_result == K4A_STREAM_RESULT_EOF)
		{
			printf("ERROR: Recording file is empty: %s\n", playback->files[i].filename);
			result = K4A_RESULT_FAILED;
			break;
		}
		else if (stream_result == K4A_STREAM_RESULT_FAILED)
		{
			printf("ERROR: Failed to read first capture from file: %s\n", playback->files[i].filename);
			result = K4A_RESULT_FAILED;
			break;
		}
	}
	return (result == K4A_RESULT_FAILED) ? 0 : 1;
}
//End DU Modification

int main(int argc, char** argv)
{
	bool master_sub = get_master_subordinate_input();

	if (master_sub && argc < 3)
	{
		printf("Usage: playback_external_sync.exe <master.mkv> <sub1.mkv>...\n");
		return 1;
	}
	else if (argc < 2)
	{
		printf("Usage: playback_external_sync.exe <video_file.mkv>\n");
	}

	playback_info playback;

	if (get_playback_info(argc, argv, &playback, master_sub))
	{
		int state = get_state_user_input();
		bool bounding = get_bounding_input();
		if (state == 0) {
			return process_mkv_offline_json(playback, bounding) ? 0 : -1;
		}
		else if (state == 1) {
			return process_mkv_offline_point(playback, bounding) ? 0 : -1;
		}
		else {
			return process_mkv_offline_both(playback, bounding) ? 0 : -1;
		}
	}
}