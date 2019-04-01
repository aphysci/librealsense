// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <mutex>
#include <fstream>

#include <math.h>
#include <float.h>
#include <types.h>
#include <conio.h>

inline rs2_quaternion quaternion_exp(rs2_vector v)
{
    float x = v.x/2, y = v.y/2, z = v.z/2, th2, th = sqrtf(th2 = x*x + y*y + z*z);
    float c = cosf(th), s = th2 < sqrtf(120*FLT_EPSILON) ? 1-th2/6 : sinf(th)/th;
    rs2_quaternion Q = { s*x, s*y, s*z, c };
    return Q;
}

inline rs2_quaternion quaternion_multiply(rs2_quaternion a, rs2_quaternion b)
{
    rs2_quaternion Q = {
        a.x * b.w + a.w * b.x - a.z * b.y + a.y * b.z,
        a.y * b.w + a.z * b.x + a.w * b.y - a.x * b.z,
        a.z * b.w - a.y * b.x + a.x * b.y + a.w * b.z,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    };
    return Q;
}

rs2_pose predict_pose(rs2_pose & pose, float dt_s)
{
    rs2_pose P = pose;
    P.translation.x = dt_s * (dt_s/2 * pose.acceleration.x + pose.velocity.x) + pose.translation.x;
    P.translation.y = dt_s * (dt_s/2 * pose.acceleration.y + pose.velocity.y) + pose.translation.y;
    P.translation.z = dt_s * (dt_s/2 * pose.acceleration.z + pose.velocity.z) + pose.translation.z;
    rs2_vector W = {
            dt_s * (dt_s/2 * pose.angular_acceleration.x + pose.angular_velocity.x),
            dt_s * (dt_s/2 * pose.angular_acceleration.y + pose.angular_velocity.y),
            dt_s * (dt_s/2 * pose.angular_acceleration.z + pose.angular_velocity.z),
    };
    P.rotation = quaternion_multiply(quaternion_exp(W), pose.rotation);
    return P;
}

int main(int argc, char * argv[]) try {
	using namespace std;
	rs2::log_to_file(RS2_LOG_SEVERITY_DEBUG, "C:\\temp\\logfile");
	const char *MAPFILE = "c:\\temp\\mapfile";

	rs2::context ctx;

	bool relocalizing = false;
	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe(ctx);

	auto devices = ctx.query_devices(RS2_PRODUCT_LINE_ANY_INTEL);
	while (devices.size() == 0) {
		cout << "No device found" << endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
	auto pose_snr = devices.front().first<rs2::pose_sensor>();


	// Create a configuration for configuring the pipeline with a non default profile
	rs2::config cfg;
	// Add pose stream
	cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);

	bool print_pose = false;
	// Define frame callback 
	// The callback is executed on a sensor thread and can be called simultaneously from multiple sensors
	// Therefore any modification to common memory should be done under lock
	std::mutex mutex;
	auto callback = [&](const rs2::frame& frame) {
		std::lock_guard<std::mutex> lock(mutex);
		if (rs2::pose_frame fp = frame.as<rs2::pose_frame>()) {
			rs2_pose pose_data = fp.get_pose_data();
			auto now = std::chrono::system_clock::now().time_since_epoch();
			double now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
			double pose_time_ms = fp.get_timestamp();
			float dt_s = static_cast<float>(std::max(0., (now_ms - pose_time_ms) / 1000.));
			rs2_pose predicted_pose = predict_pose(pose_data, dt_s);
			if (print_pose) {
				std::cout << "Predicted " << std::fixed << std::setprecision(3) << dt_s * 1000 << "ms " <<
					"Confidence: " << pose_data.tracker_confidence << " T: " <<
					predicted_pose.translation.x << " " <<
					predicted_pose.translation.y << " " <<
					predicted_pose.translation.z << " (meters)   \r" << endl;
				print_pose = false;
			}

		}
	};

	std::ifstream in(MAPFILE, std::ifstream::ate | std::ifstream::binary);
	if (in.good()) {
		auto sz = in.tellg();
		in.seekg(0);
		vector<byte> lmap_buf(sz);
		in.read((char *)lmap_buf.data(), sz);
		bool status = pose_snr.import_localization_map(lmap_buf);
		cout << "Import map status = " << status << endl;
		relocalizing = true;
	} else {
		cout << "Will create mapfile: " << MAPFILE << endl;
	}

	// Start streaming through the callback with default recommended configuration
	rs2::pipeline_profile profiles = pipe.start(cfg, callback);
	std::cout << "started thread\nPress 'x' to stop\n" << endl;

	while (true) {
		char c = getch();
		if (c == 'x') break;
		if (c == 'p') {
			print_pose = true;
			continue;
		}
		if (relocalizing) {
			rs2_vector pos;
			rs2_quaternion orient;
			bool is_there = pose_snr.get_static_node("origin", pos, orient);
			cout << endl << "reloc: " << is_there << " pos = (" << pos.x << "," << pos.y << "," << pos.z << ")" << endl;
		}
	}


	//cin.ignore();
	if (!relocalizing) {
		std::lock_guard<std::mutex> lock(mutex);

		float temp = pose_snr.get_option(rs2_option::RS2_OPTION_ASIC_TEMPERATURE);
		std::cout << std::endl << "setting origin.   temp = " << temp << std::endl;
		bool status = pose_snr.set_static_node("origin", { 0, 0, 0 }, { 0, 0, 0, 1 });
		status = pose_snr.set_static_node("one", { 1, 0, 0 }, { 0, 0, 0, 1 });
		status = pose_snr.set_static_node("two", { 0, 1, 0 }, { 0, 0, 0, 1 });
		status = pose_snr.set_static_node("three", { 0, 0, 1 }, { 0, 0, 0, 1 });
		status = pose_snr.set_static_node("-one", { -1, 0, 0 }, { 0, 0, 0, 1 });
		status = pose_snr.set_static_node("-two", { 0, -1, 0 }, { 0, 0, 0, 1 });
		status = pose_snr.set_static_node("-three", { 0, 0, -1 }, { 0, 0, 0, 1 });
		temp = pose_snr.get_option(rs2_option::RS2_OPTION_ASIC_TEMPERATURE);
		std::cout << "stopping. temp = " << temp << " stat = " << status << std::endl;
	}

	pipe.stop();
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	if (!relocalizing) {
		std::cout << "saving map: " << std::endl;
		auto map_bytes = pose_snr.export_localization_map();
		std::cout << "bytes " << map_bytes.size() << std::endl;

		ofstream mapfile(MAPFILE, ios::binary);
		mapfile.write((const char *)map_bytes.data(), map_bytes.size());
		mapfile.close();
	}

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
