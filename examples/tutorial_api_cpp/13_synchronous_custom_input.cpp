//// ------------------------- OpenPose C++ API Tutorial - Example 13 - Custom Input -------------------------
//// Synchronous mode: ideal for production integration. It provides the fastest results with respect to runtime
//// performance.
//// In this function, the user can implement its own way to create frames (e.g., reading his own folder of images).
//
//// Command-line user intraface
//#define OPENPOSE_FLAGS_DISABLE_PRODUCER
//#include <openpose/flags.hpp>
//// OpenPose dependencies
//#include <openpose/headers.hpp>
//
//// Custom OpenPose flags
//// Producer
//DEFINE_string(image_dir,                "examples/media/",
//    "Process a directory of images. Read all standard formats (jpg, png, bmp, etc.).");
//DEFINE_string(calibration_dir, "models/models/cameraParameters/flir/",
//	"Not sure why should have this line, just follow the above code structure.");
////DEFINE_string(video1, "C:\Users\qyuan\Downloads\temp-12192019143934-0000.avi",
////	"");
////DEFINE_string(video2, "C:\Users\qyuan\Downloads\temp-12192019143935-0000.avi",
////	"");
//// This worker will just read and return all the basic image file formats in a directory
////class WUserInput : public op::WorkerProducer<std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>>
////{
////public:
////    WUserInput(const std::string& directoryPath) :
////        mImageFiles{op::getFilesOnDirectory(directoryPath, op::Extensions::Images)}, // For all basic image formats
////        // If we want only e.g., "jpg" + "png" images
////        // mImageFiles{op::getFilesOnDirectory(directoryPath, std::vector<std::string>{"jpg", "png"})},
////        mCounter{0}
////    {
////        if (mImageFiles.empty())
////            op::error("No images found on: " + directoryPath, __LINE__, __FUNCTION__, __FILE__);
////    }
////
////    void initializationOnThread() {}
////
////    std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> workProducer()
////    {
////        try
////        {
////            // Close program when empty frame
////            if (mImageFiles.size() <= mCounter)
////            {
////                op::log("Last frame read and added to queue. Closing program after it is processed.",
////                        op::Priority::High);
////                // This funtion stops this worker, which will eventually stop the whole thread system once all the
////                // frames have been processed
////                this->stop();
////                return nullptr;
////            }
////            else
////            {
////                // Create new datum
////                auto datumsPtr = std::make_shared<std::vector<std::shared_ptr<op::Datum>>>();
////                datumsPtr->emplace_back();
////                auto& datumPtr = datumsPtr->at(0);
////                datumPtr = std::make_shared<op::Datum>();
////
////                // Fill datum
////                datumPtr->cvInputData = cv::imread(mImageFiles.at(mCounter++));
////
////                // If empty frame -> return nullptr
////                if (datumPtr->cvInputData.empty())
////                {
////                    op::log("Empty frame detected on path: " + mImageFiles.at(mCounter-1) + ". Closing program.",
////                        op::Priority::High);
////                    this->stop();
////                    datumsPtr = nullptr;
////                }
////
////                return datumsPtr;
////            }
////        }
////        catch (const std::exception& e)
////        {
////            this->stop();
////            op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
////            return nullptr;
////        }
////    }
////
////private:
////    const std::vector<std::string> mImageFiles;
////    unsigned long long mCounter;
////};
////class WUserInput : public op::WorkerProducer<std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>>
////{
////public:
////	WUserInput(const std::string& cam_ids)
////		: //mUse3d(use3d),
////		mParamReader(std::make_shared<op::CameraParameterReader>())
////	{
////		for (const auto& cam_id : cam_ids) {
////			mCams.emplace_back(std::make_shared<op::VideoCaptureReader>(cam_id));
////		}
////		if (1==1) {
////			mParamReader->readParameters(FLAGS_calibration_dir);
////			mIntrinsics = mParamReader->getCameraIntrinsics();
////			mExtrinsics = mParamReader->getCameraExtrinsics();
////			mMatrices = mParamReader->getCameraMatrices();
////		}
////	}
////
////	void initializationOnThread() {}
////
////	std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> workProducer()
////	{
////		try
////		{
////			std::lock_guard<std::mutex> g(lock);
////			if (mBlocked.empty()) {
////
////				for (size_t i = 0; i < mCams.size(); i++) {
////					auto datumsPtr = std::make_shared<std::vector<std::shared_ptr<op::Datum>>>();
////					// Create new datum
////					datumsPtr->emplace_back();
////					auto& datum = datumsPtr->back();
////					datum = std::make_shared<op::Datum>();
////
////					// Fill datum
////					datum->cvInputData = mCams[i]->getFrame();
////					datum->cvOutputData = datum->cvInputData;
////					datum->subId = i;
////					datum->subIdMax = mCams.size() - 1;
////					if (mUse3d) {
////						datum->cameraIntrinsics = mIntrinsics[i];
////						datum->cameraExtrinsics = mExtrinsics[i];
////						datum->cameraMatrix = mMatrices[i];
////					}
////
////					// If empty frame -> return nullptr
////					if (datum->cvInputData.empty())
////					{
////						this->stop();
////						return nullptr;
////					}
////
////					mBlocked.push(datumsPtr);
////				}
////			}
////
////			auto ret = mBlocked.front();
////			mBlocked.pop();
////			return ret;
////		}
////		catch (const std::exception & e)
////		{
////			this->stop();
////			op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
////			return nullptr;
////		}
////	}
////
////private:
////	bool mUse3d;
////	std::shared_ptr<op::CameraParameterReader> mParamReader;
////	std::vector<cv::Mat> mIntrinsics;
////	std::vector<cv::Mat> mExtrinsics;
////	std::vector<cv::Mat> mMatrices;
////	std::vector<std::shared_ptr<op::VideoCaptureReader>> mCams;
////	std::queue<std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>> mBlocked;
////	std::mutex lock;
////};
//class WUserInput : public op::WorkerProducer<std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>>
//{
//public:
//	WUserInput(std::vector<std::string> video_paths, bool use3d)
//		: mUse3d(use3d),
//		mParamReader(std::make_shared<op::CameraParameterReader>())
//	{
//		for (std::string& video_path : video_paths) {
//			mCams.emplace_back(std::make_shared<op::VideoReader>(video_path));
//		}
//		if (use3d) {
//			mParamReader->readParameters(FLAGS_calibration_dir);
//			mIntrinsics = mParamReader->getCameraIntrinsics();
//			mExtrinsics = mParamReader->getCameraExtrinsics();
//			mMatrices = mParamReader->getCameraMatrices();
//		}
//	}
//
//	void initializationOnThread() {}
//
//	std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> workProducer()
//	{
//		try
//		{
//			std::lock_guard<std::mutex> g(lock);
//			if (mBlocked.empty()) {
//
//				for (size_t i = 0; i < mCams.size(); i++) {
//					auto datumsPtr = std::make_shared<std::vector<std::shared_ptr<op::Datum>>>();
//					// Create new datum
//					datumsPtr->emplace_back();
//					auto& datum = datumsPtr->back();
//					datum = std::make_shared<op::Datum>();
//
//					// Fill datum
//					datum->cvInputData = mCams[i]->getFrame();
//					datum->cvOutputData = datum->cvInputData;
//					datum->subId = i;
//					datum->subIdMax = mCams.size() - 1;
//					if (mUse3d) {
//						datum->cameraIntrinsics = mIntrinsics[i];
//						datum->cameraExtrinsics = mExtrinsics[i];
//						datum->cameraMatrix = mMatrices[i];
//					}
//
//					// If empty frame -> return nullptr
//					if (datum->cvInputData.empty())
//					{
//						this->stop();
//						return nullptr;
//					}
//
//					mBlocked.push(datumsPtr);
//				}
//			}
//
//			auto ret = mBlocked.front();
//			mBlocked.pop();
//			return ret;
//		}
//		catch (const std::exception & e)
//		{
//			this->stop();
//			op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
//			return nullptr;
//		}
//	}
//
//private:
//	bool mUse3d;
//	std::shared_ptr<op::CameraParameterReader> mParamReader;
//	std::vector<cv::Mat> mIntrinsics;
//	std::vector<cv::Mat> mExtrinsics;
//	std::vector<cv::Mat> mMatrices;
//	std::vector<std::shared_ptr<op::VideoReader>> mCams;
//	std::queue<std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>> mBlocked;
//	std::mutex lock;
//};
//
//void configureWrapper(op::Wrapper& opWrapper)
//{
//    try
//    {
//        // Configuring OpenPose
//
//        // logging_level
//        op::check(0 <= FLAGS_logging_level && FLAGS_logging_level <= 255, "Wrong logging_level value.",
//                  __LINE__, __FUNCTION__, __FILE__);
//        op::ConfigureLog::setPriorityThreshold((op::Priority)FLAGS_logging_level);
//        op::Profiler::setDefaultX(FLAGS_profile_speed);
//
//        // Applying user defined configuration - GFlags to program variables
//        // outputSize
//        const auto outputSize = op::flagsToPoint(FLAGS_output_resolution, "-1x-1");
//        // netInputSize
//        const auto netInputSize = op::flagsToPoint(FLAGS_net_resolution, "-1x368");
//        // faceNetInputSize
//        const auto faceNetInputSize = op::flagsToPoint(FLAGS_face_net_resolution, "368x368 (multiples of 16)");
//        // handNetInputSize
//        const auto handNetInputSize = op::flagsToPoint(FLAGS_hand_net_resolution, "368x368 (multiples of 16)");
//        // poseMode
//        const auto poseMode = op::flagsToPoseMode(FLAGS_body);
//        // poseModel
//        const auto poseModel = op::flagsToPoseModel(FLAGS_model_pose);
//        // JSON saving
//        if (!FLAGS_write_keypoint.empty())
//            op::log("Flag `write_keypoint` is deprecated and will eventually be removed."
//                    " Please, use `write_json` instead.", op::Priority::Max);
//        // keypointScaleMode
//        const auto keypointScaleMode = op::flagsToScaleMode(FLAGS_keypoint_scale);
//        // heatmaps to add
//        const auto heatMapTypes = op::flagsToHeatMaps(FLAGS_heatmaps_add_parts, FLAGS_heatmaps_add_bkg,
//                                                      FLAGS_heatmaps_add_PAFs);
//        const auto heatMapScaleMode = op::flagsToHeatMapScaleMode(FLAGS_heatmaps_scale);
//        // >1 camera view?
//        // const auto multipleView = (FLAGS_3d || FLAGS_3d_views > 1 || FLAGS_flir_camera);
//        const auto multipleView = false;
//        // Face and hand detectors
//        const auto faceDetector = op::flagsToDetector(FLAGS_face_detector);
//        const auto handDetector = op::flagsToDetector(FLAGS_hand_detector);
//        // Enabling Google Logging
//        const bool enableGoogleLogging = true;
//
//        // Initializing the user custom classes
//        // Frames producer (e.g., video, webcam, ...)
//        auto wUserInput = std::make_shared<WUserInput>(FLAGS_image_dir);
//        // Add custom processing
//        const auto workerInputOnNewThread = true;
//        opWrapper.setWorker(op::WorkerType::Input, wUserInput, workerInputOnNewThread);
//
//        // Pose configuration (use WrapperStructPose{} for default and recommended configuration)
//        const op::WrapperStructPose wrapperStructPose{
//            poseMode, netInputSize, outputSize, keypointScaleMode, FLAGS_num_gpu, FLAGS_num_gpu_start,
//            FLAGS_scale_number, (float)FLAGS_scale_gap, op::flagsToRenderMode(FLAGS_render_pose, multipleView),
//            poseModel, !FLAGS_disable_blending, (float)FLAGS_alpha_pose, (float)FLAGS_alpha_heatmap,
//            FLAGS_part_to_show, FLAGS_model_folder, heatMapTypes, heatMapScaleMode, FLAGS_part_candidates,
//            (float)FLAGS_render_threshold, FLAGS_number_people_max, FLAGS_maximize_positives, FLAGS_fps_max,
//            FLAGS_prototxt_path, FLAGS_caffemodel_path, (float)FLAGS_upsampling_ratio, enableGoogleLogging};
//        opWrapper.configure(wrapperStructPose);
//        // Face configuration (use op::WrapperStructFace{} to disable it)
//        const op::WrapperStructFace wrapperStructFace{
//            FLAGS_face, faceDetector, faceNetInputSize,
//            op::flagsToRenderMode(FLAGS_face_render, multipleView, FLAGS_render_pose),
//            (float)FLAGS_face_alpha_pose, (float)FLAGS_face_alpha_heatmap, (float)FLAGS_face_render_threshold};
//        opWrapper.configure(wrapperStructFace);
//        // Hand configuration (use op::WrapperStructHand{} to disable it)
//        const op::WrapperStructHand wrapperStructHand{
//            FLAGS_hand, handDetector, handNetInputSize, FLAGS_hand_scale_number, (float)FLAGS_hand_scale_range,
//            op::flagsToRenderMode(FLAGS_hand_render, multipleView, FLAGS_render_pose), (float)FLAGS_hand_alpha_pose,
//            (float)FLAGS_hand_alpha_heatmap, (float)FLAGS_hand_render_threshold};
//        opWrapper.configure(wrapperStructHand);
//        // Extra functionality configuration (use op::WrapperStructExtra{} to disable it)
//        const op::WrapperStructExtra wrapperStructExtra{
//            FLAGS_3d, FLAGS_3d_min_views, FLAGS_identification, FLAGS_tracking, FLAGS_ik_threads};
//        opWrapper.configure(wrapperStructExtra);
//        // Output (comment or use default argument to disable any output)
//        const op::WrapperStructOutput wrapperStructOutput{
//            FLAGS_cli_verbose, FLAGS_write_keypoint, op::stringToDataFormat(FLAGS_write_keypoint_format),
//            FLAGS_write_json, FLAGS_write_coco_json, FLAGS_write_coco_json_variants, FLAGS_write_coco_json_variant,
//            FLAGS_write_images, FLAGS_write_images_format, FLAGS_write_video, FLAGS_write_video_fps,
//            FLAGS_write_video_with_audio, FLAGS_write_heatmaps, FLAGS_write_heatmaps_format, FLAGS_write_video_3d,
//            FLAGS_write_video_adam, FLAGS_write_bvh, FLAGS_udp_host, FLAGS_udp_port};
//        opWrapper.configure(wrapperStructOutput);
//        // GUI (comment or use default argument to disable any visual output)
//        const op::WrapperStructGui wrapperStructGui{
//            op::flagsToDisplayMode(FLAGS_display, FLAGS_3d), !FLAGS_no_gui_verbose, FLAGS_fullscreen};
//        opWrapper.configure(wrapperStructGui);
//        // Set to single-thread (for sequential processing and/or debugging and/or reducing latency)
//        if (FLAGS_disable_multi_thread)
//            opWrapper.disableMultiThreading();
//    }
//    catch (const std::exception& e)
//    {
//        op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
//    }
//}
//
//int tutorialApiCpp()
//{
//    try
//    {
//        op::log("Starting OpenPose demo...", op::Priority::High);
//        const auto opTimer = op::getTimerInit();
//
//        // OpenPose wrapper
//        op::log("Configuring OpenPose...", op::Priority::High);
//        op::Wrapper opWrapper;
//        configureWrapper(opWrapper);
//
//        // Start, run, and stop processing - exec() blocks this thread until OpenPose wrapper has finished
//        op::log("Starting thread(s)...", op::Priority::High);
//        opWrapper.exec();
//
//        // Measuring total time
//        op::printTime(opTimer, "OpenPose demo successfully finished. Total time: ", " seconds.", op::Priority::High);
//
//        // Return
//        return 0;
//    }
//    catch (const std::exception& e)
//    {
//        return -1;
//    }
//}
//
//int main(int argc, char *argv[])
//{
//    // Parsing command line flags
//    gflags::ParseCommandLineFlags(&argc, &argv, true);
//
//    // Running tutorialApiCpp
//    return tutorialApiCpp();
//}

//3d synchronous input
// Command-line user intraface
#define OPENPOSE_FLAGS_DISABLE_PRODUCER
#include <openpose/flags.hpp>
// OpenPose dependencies
#include <openpose/headers.hpp>
#include <opencv2/opencv.hpp> 
#include <windows.h>
//#include <opencv2/core/core.hpp>
DEFINE_bool(no_display, false,
	"Enable to disable the visual display.");
// Custom OpenPose flags
// Producer

//Add cameras in order (for cameras 1 and 2: 2112638 then 2112645, for 2 and 3: )
//DEFINE_string(video1, "C:\\Users\\yqtl\\Videos\\temp-12192019143935-0000.avi",
//	"");
//DEFINE_string(video2, "C:\\Users\\yqtl\\Videos\\temp-12192019143934-0000.avi",
//	"");
DEFINE_string(video1, "C:\\Users\\yqtl\\Videos\\temp1.avi",
	"");
DEFINE_string(video2, "C:\\Users\\yqtl\\Videos\\temp2.avi",
	"");
DEFINE_string(camera_parameter_path, "./models/cameraParameters/flir/", "");
//DEFINE_string(write_video, false, "");


// This worker will just read and return all the basic image file formats in a directory
class WUserInput : public op::WorkerProducer<std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>>
{
public:
	WUserInput(std::vector<std::string> video_paths, bool use3d)
		: mUse3d(use3d),
		mParamReader(std::make_shared<op::CameraParameterReader>())
	{
		for (std::string& video_path : video_paths) {
			mCams.emplace_back(std::make_shared<op::VideoReader>(video_path));
		}
		if (use3d) {
			mParamReader->readParameters(FLAGS_camera_parameter_path);
			mIntrinsics = mParamReader->getCameraIntrinsics();
			mExtrinsics = mParamReader->getCameraExtrinsics();
			mMatrices = mParamReader->getCameraMatrices();
		}
	}

	void initializationOnThread() {}

	std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> workProducer()
	{
		try
		{
			std::lock_guard<std::mutex> g(lock);
			if (mBlocked.empty()) {

				for (size_t i = 0; i < mCams.size(); i++) {
					auto datumsPtr = std::make_shared<std::vector<std::shared_ptr<op::Datum>>>();
					// Create new datum
					datumsPtr->emplace_back();
					auto& datum = datumsPtr->back();
					datum = std::make_shared<op::Datum>();

					// Fill datum
					datum->cvInputData = mCams[i]->getFrame();
					datum->cvOutputData = datum->cvInputData;
					datum->subId = i;
					datum->subIdMax = mCams.size() - 1;
					if (mUse3d) {
						datum->cameraIntrinsics = mIntrinsics[i];
						datum->cameraExtrinsics = mExtrinsics[i];
						datum->cameraMatrix = mMatrices[i];
					}

					// If empty frame -> return nullptr
					if (datum->cvInputData.empty())
					{
						this->stop();
						return nullptr;
					}
					Sleep(100);
					mBlocked.push(datumsPtr);
				}
			}

			auto ret = mBlocked.front();
			mBlocked.pop();
			return ret;
		}
		catch (const std::exception& e)
		{
			this->stop();
			op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
			return nullptr;
		}
	}

private:
	bool mUse3d;
	std::shared_ptr<op::CameraParameterReader> mParamReader;
	std::vector<op::Matrix> mIntrinsics;
	std::vector<op::Matrix> mExtrinsics;
	std::vector<op::Matrix> mMatrices;
	std::vector<std::shared_ptr<op::VideoReader>> mCams;
	std::queue<std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>> mBlocked;
	std::mutex lock;
};
void configureWrapper(op::Wrapper& opWrapper)
{
	try
	{
		// Configuring OpenPose

		// logging_level
		op::checkBool(
			0 <= FLAGS_logging_level && FLAGS_logging_level <= 255, "Wrong logging_level value.",
			__LINE__, __FUNCTION__, __FILE__);
		op::ConfigureLog::setPriorityThreshold((op::Priority)FLAGS_logging_level);
		op::Profiler::setDefaultX(FLAGS_profile_speed);

		// Applying user defined configuration - GFlags to program variables
		// outputSize
		const auto outputSize = op::flagsToPoint(op::String(FLAGS_output_resolution), "-1x-1");
		// netInputSize
		const auto netInputSize = op::flagsToPoint(op::String(FLAGS_net_resolution), "-1x368");
		// faceNetInputSize
		const auto faceNetInputSize = op::flagsToPoint(op::String(FLAGS_face_net_resolution), "368x368 (multiples of 16)");
		// handNetInputSize
		const auto handNetInputSize = op::flagsToPoint(op::String(FLAGS_hand_net_resolution), "368x368 (multiples of 16)");
		// poseMode
		const auto poseMode = op::flagsToPoseMode(FLAGS_body);
		// poseModel
		const auto poseModel = op::flagsToPoseModel(op::String(FLAGS_model_pose));
		// JSON saving
		if (!FLAGS_write_keypoint.empty())
			op::opLog(
				"Flag `write_keypoint` is deprecated and will eventually be removed. Please, use `write_json`"
				" instead.", op::Priority::Max);
		// keypointScaleMode
		const auto keypointScaleMode = op::flagsToScaleMode(FLAGS_keypoint_scale);
		// heatmaps to add
		const auto heatMapTypes = op::flagsToHeatMaps(FLAGS_heatmaps_add_parts, FLAGS_heatmaps_add_bkg,
			FLAGS_heatmaps_add_PAFs);
		const auto heatMapScaleMode = op::flagsToHeatMapScaleMode(FLAGS_heatmaps_scale);
		// >1 camera view?
		const auto multipleView = (FLAGS_3d || FLAGS_3d_views > 1);
		// Face and hand detectors
		const auto faceDetector = op::flagsToDetector(FLAGS_face_detector);
		const auto handDetector = op::flagsToDetector(FLAGS_hand_detector);
		// Enabling Google Logging
		const bool enableGoogleLogging = true;


		// Initializing the user custom classes
		// Frames producer (e.g., video, webcam, ...)
		std::vector<std::string> paths;
		paths.push_back(FLAGS_video1);
		paths.push_back(FLAGS_video2);
		auto wUserInput = std::make_shared<WUserInput>(paths, true);


		// Add custom processing
		const auto workerInputOnNewThread = true;
		opWrapper.setWorker(op::WorkerType::Input, wUserInput, workerInputOnNewThread);



		// Pose configuration (use WrapperStructPose{} for default and recommended configuration)
		const op::WrapperStructPose wrapperStructPose{
			poseMode, netInputSize, outputSize, keypointScaleMode, FLAGS_num_gpu, FLAGS_num_gpu_start,
			FLAGS_scale_number, (float)FLAGS_scale_gap, op::flagsToRenderMode(FLAGS_render_pose, multipleView),
			poseModel, !FLAGS_disable_blending, (float)FLAGS_alpha_pose, (float)FLAGS_alpha_heatmap,
			FLAGS_part_to_show, op::String(FLAGS_model_folder), heatMapTypes, heatMapScaleMode, FLAGS_part_candidates,
			(float)FLAGS_render_threshold, FLAGS_number_people_max, FLAGS_maximize_positives, FLAGS_fps_max,
			op::String(FLAGS_prototxt_path), op::String(FLAGS_caffemodel_path),
			(float)FLAGS_upsampling_ratio, enableGoogleLogging };
		opWrapper.configure(wrapperStructPose);
		// Face configuration (use op::WrapperStructFace{} to disable it)
		const op::WrapperStructFace wrapperStructFace{
			FLAGS_face, faceDetector, faceNetInputSize,
			op::flagsToRenderMode(FLAGS_face_render, multipleView, FLAGS_render_pose),
			(float)FLAGS_face_alpha_pose, (float)FLAGS_face_alpha_heatmap, (float)FLAGS_face_render_threshold };
		opWrapper.configure(wrapperStructFace);
		// Hand configuration (use op::WrapperStructHand{} to disable it)
		const op::WrapperStructHand wrapperStructHand{
			FLAGS_hand, handDetector, handNetInputSize, FLAGS_hand_scale_number, (float)FLAGS_hand_scale_range,
			op::flagsToRenderMode(FLAGS_hand_render, multipleView, FLAGS_render_pose), (float)FLAGS_hand_alpha_pose,
			(float)FLAGS_hand_alpha_heatmap, (float)FLAGS_hand_render_threshold };
		opWrapper.configure(wrapperStructHand);
		// Extra functionality configuration (use op::WrapperStructExtra{} to disable it)
		const op::WrapperStructExtra wrapperStructExtra{
			FLAGS_3d, FLAGS_3d_min_views, FLAGS_identification, FLAGS_tracking, FLAGS_ik_threads };
		opWrapper.configure(wrapperStructExtra);
		// Output (comment or use default argument to disable any output)
		const op::WrapperStructOutput wrapperStructOutput{
			FLAGS_cli_verbose, op::String(FLAGS_write_keypoint), op::stringToDataFormat(FLAGS_write_keypoint_format),
			op::String(FLAGS_write_json), op::String(FLAGS_write_coco_json), FLAGS_write_coco_json_variants,
			FLAGS_write_coco_json_variant, op::String(FLAGS_write_images), op::String(FLAGS_write_images_format),
			op::String(FLAGS_write_video), FLAGS_write_video_fps, FLAGS_write_video_with_audio,
			op::String(FLAGS_write_heatmaps), op::String(FLAGS_write_heatmaps_format), op::String(FLAGS_write_video_3d),
			op::String(FLAGS_write_video_adam), op::String(FLAGS_write_bvh), op::String(FLAGS_udp_host),
			op::String(FLAGS_udp_port) };
		opWrapper.configure(wrapperStructOutput);
		// No GUI. Equivalent to: opWrapper.configure(op::WrapperStructGui{});
		const op::WrapperStructGui wrapperStructGui{
			op::flagsToDisplayMode(FLAGS_display, FLAGS_3d), !FLAGS_no_gui_verbose, FLAGS_fullscreen };
		opWrapper.configure(wrapperStructGui);
		// Set to single-thread (for sequential processing and/or debugging and/or reducing latency)
		if (FLAGS_disable_multi_thread)
			opWrapper.disableMultiThreading();
	}
	catch (const std::exception & e)
	{
		op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
	}
}
//void printKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
//{
//	try
//	{
//		// Example: How to use the pose keypoints
//		if (datumsPtr != nullptr && !datumsPtr->empty())
//		{
//			// Alternative 1
//			op::log("Body keypoints: " + datumsPtr->at(0)->poseKeypoints.toString(), op::Priority::High);
//
//			// // Alternative 2
//			// op::log(datumsPtr->at(0).poseKeypoints, op::Priority::High);
//
//			// // Alternative 3
//			// std::cout << datumsPtr->at(0).poseKeypoints << std::endl;
//
//			// // Alternative 4 - Accesing each element of the keypoints
//			// op::log("\nKeypoints:", op::Priority::High);
//			// const auto& poseKeypoints = datumsPtr->at(0).poseKeypoints;
//			// op::log("Person pose keypoints:", op::Priority::High);
//			// for (auto person = 0 ; person < poseKeypoints.getSize(0) ; person++)
//			// {
//			//     op::log("Person " + std::to_string(person) + " (x, y, score):", op::Priority::High);
//			//     for (auto bodyPart = 0 ; bodyPart < poseKeypoints.getSize(1) ; bodyPart++)
//			//     {
//			//         std::string valueToPrint;
//			//         for (auto xyscore = 0 ; xyscore < poseKeypoints.getSize(2) ; xyscore++)
//			//             valueToPrint += std::to_string(   poseKeypoints[{person, bodyPart, xyscore}]   ) + " ";
//			//         op::log(valueToPrint, op::Priority::High);
//			//     }
//			// }
//			// op::log(" ", op::Priority::High);
//		}
//		else
//			op::log("Nullptr or empty datumsPtr found.", op::Priority::High);
//	}
//	catch (const std::exception & e)
//	{
//		op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
//	}
//}

int tutorialApiCpp()
{
	try
	{
		op::opLog("Starting OpenPose...", op::Priority::High);
		const auto opTimer = op::getTimerInit();

		// OpenPose wrapper
		op::opLog("Configuring OpenPose...", op::Priority::High);
		op::Wrapper opWrapper{op::ThreadManagerMode::Synchronous};

		configureWrapper(opWrapper);

		// Start, run, and stop processing - exec() blocks this thread until OpenPose wrapper has finished
		op::opLog("Starting thread(s)...", op::Priority::High);
		opWrapper.exec();

		// Measuring total time
		op::printTime(opTimer, "OpenPose successfully finished. Total time: ", " seconds.", op::Priority::High);

		// Return
		return 0;
	}
	catch (const std::exception & e)
	{
		return -1;
	}
}

int main(int argc, char* argv[])
{
	// Parsing command line flags
	gflags::ParseCommandLineFlags(&argc, &argv, true);

	// Running tutorialApiCpp
	return tutorialApiCpp();
}
