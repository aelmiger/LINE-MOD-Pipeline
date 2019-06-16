
#include <vector>
#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ccalib.hpp>
#include <opencv2/rgbd.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/surface_matching.hpp>
#include <opencv2/surface_matching/ppf_helpers.hpp>
#include <opencv2/videoio.hpp>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>


#include <fstream>

#include "opengl_render.h"
#include "model_buffer.h"
#include "utility.h"
#include "defines.h"
#include "kinect2.h"

#include "high_level_linemod.h"
#include "high_level_linemod_icp.h"
#include "Template_Generator.h"

#ifdef _DEBUG
#pragma comment(lib, "SDL2.lib")
#pragma comment(lib, "glew32s.lib")
#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "opencv_core410d.lib")
#pragma comment(lib, "opencv_imgproc410d.lib")
#pragma comment(lib, "opencv_rgbd410d.lib")
#pragma comment(lib, "opencv_imgcodecs410d.lib")
#pragma comment(lib, "opencv_highgui410d.lib")
#pragma comment(lib, "opencv_videoio410d.lib")
#pragma comment(lib, "opencv_ccalib410d.lib")
#pragma comment(lib, "opencv_calib3d410d.lib")
#pragma comment(lib, "opencv_surface_matching410d.lib")
#pragma comment(lib, "opencv_videoio410d.lib")
#pragma comment(lib, "freenect2.lib")



#else
#pragma comment(lib, "SDL2.lib")
#pragma comment(lib, "glew32s.lib")
#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "opencv_core410.lib")
#pragma comment(lib, "opencv_imgproc410.lib")
#pragma comment(lib, "opencv_rgbd410.lib")
#pragma comment(lib, "opencv_imgcodecs410.lib")
#pragma comment(lib, "opencv_highgui410.lib")
#pragma comment(lib, "opencv_videoio410.lib")
#pragma comment(lib, "opencv_ccalib410.lib")
#pragma comment(lib, "opencv_calib3d410.lib")
#pragma comment(lib, "opencv_surface_matching410.lib")
#pragma comment(lib, "opencv_videoio410.lib")
#pragma comment(lib, "freenect2.lib")


#endif

//const bool readDetectorFromFile = true;
//
/////////CAMERA PARAMETER LINE_MOD BENCHMARK
////const float32 fx = 572.41140;
////const float32 cx = 325.26110;
////const float32 fy = 573.57043;
////const float32 cy = 242.04899;
/////////CAMERA PARAMETER KINECT V2
//const float32 fx = 1044.871;
//const float32 cx = 309.26110;
//const float32 fy = 1045.69141;
//const float32 cy = 239.04899;
//cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
//cv::Mat distortionCoefficients = (cv::Mat1d(1, 5) << -2.7167827743927644e-03, 2.0942424424199252e-01,
//	1.1120545920170163e-03, -6.6420567497010334e-03, 0.);
//const uint16 videoWidth = 640;
//const uint16 videoHeight = 480;
//
/////////TEMPLATE CREATION SETTINGS
//const float64 startDistance = 800;
//const float64 endDistance = 1500;
//const float64 stepSize = 50.0f;
//const uint8 subdivisions = 2;
//const float32 angleStart = -45;
//const float32 angleStop = 45;
//const float32 angleStep = 15;
//
/////////MODEL FILES
//const std::string modelFileEnding = ".bmf";
//std::string modelFolder = "models/";

int main() {

	Template_Generator templateGen(CameraParameters, TemplateGenerationSettings);
	//OpenGLRender opengl(videoWidth, videoHeight,cameraMatrix);
	//lineMOD::HighLevelLinemod line(false, videoWidth, videoHeight, angleStart, angleStop, angleStep, stepSize,cameraMatrix);
	//lineMODIcp::HighLevelLinemodIcp icp(5, 0.1f, 3.5f, 8,10);

	//std::vector<ModelBuffer> modBuff;

	//////////Read available Model Files
	//std::vector<std::string> modelFiles;
	//filesInDirectory(modelFiles, modelFolder, modelFileEnding);


	//if (!readDetectorFromFile){

	//	for (size_t i = 0; i < modelFiles.size(); i++)
	//	{
	//		Model tmp;
	//		opengl.readModelFile(modelFolder + modelFiles[i], tmp);
	//		std::vector<glm::vec3> tempVert;
	//		tempVert = zipVectors(tmp.vertices, tmp.colors);
	//		modBuff.push_back(ModelBuffer(tempVert.data(), tmp.numVertices, tmp.indices.data(), tmp.numIndices, sizeof(tmp.indices[0])));

	//		for (float32 sphereRadius = startDistance; sphereRadius <= endDistance; sphereRadius = sphereRadius + stepSize)
	//		{
	//			uint16 numberOfIterations = std::floor((endDistance - startDistance + stepSize)/stepSize);

	//			CameraViewPoints camPoints(sphereRadius);
	//			//CameraViewPoints camPoints(sphereRadius, subdivisions);
	//			std::vector<glm::vec3> camVertices;
	//			camVertices = camPoints.getVertices();
	//			uint32 numCameraVertices = camVertices.size();
	//			for (size_t j = 0; j < numCameraVertices; j++)
	//			{

	//				printProgBar((((j + 1) * 100) / numCameraVertices) / numberOfIterations + (sphereRadius -startDistance)/stepSize * 100 / numberOfIterations, modelFiles[i]);

	//				if (!(camVertices[j].y >= 0)) {
	//					continue;
	//				}

	//				opengl.renderDepthToFrontBuff(&modBuff[i], glm::vec3(camVertices[j]), 0.0f, 0.0f, 0.0f);
	//				cv::Mat depth = opengl.getDepthImgFromBuff();
	//				opengl.renderColorToFrontBuff(&modBuff[i], glm::vec3(camVertices[j]), 0.0f, 0.0f, 0.0f);
	//				cv::Mat color = opengl.getColorImgFromBuff();
	//				std::vector<cv::Mat> images;
	//				images.push_back(color);
	//				images.push_back(depth);
	//				line.addTemplate(images,modelFiles[i],camVertices[j]);


	//			}

	//		}
	//	}
	//	line.writeLinemod();

	//}

	else {
		for (size_t i = 0; i < modelFiles.size(); i++)
		{
			Model tmp;

			opengl.readModelFile(modelFolder + modelFiles[i], tmp);
			std::vector<glm::vec3> tempVert;
			tempVert = zipVectors(tmp.vertices, tmp.colors);
			modBuff.push_back(ModelBuffer(tempVert.data(), tmp.numVertices, tmp.indices.data(), tmp.numIndices, sizeof(tmp.indices[0])));
		}
		line.readLinemod();
		std::vector<cv::String> ids = line.getClassIds();
		int num_classes = line.getNumClasses();
		printf("Loaded with %d classes and %d templates\n",
			num_classes, line.getNumTemplates());
		if (!ids.empty())
		{
			printf("Class ids:\n");
			std::copy(ids.begin(), ids.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
		}

	}

	////////////////ONLY RELEVANT FOR BENCHMARK
	//Model tmp;
	int counter = 0;
	//float64 accumDiff = 0;
	//opengl.readModelFile(modelFolder + modelFiles[0], tmp); //TODO welches file
	///////////////



	std::vector<cv::Mat> inputImg;
	cv::Mat colorImg;
	cv::Mat depthImg;
	//Kinect2 kin2;
	//cv::VideoCapture cap(0);
	cv::VideoCapture sequence("data/color%0d.jpg");
	//glPolygonMode(GL_FRONT, GL_LINE);


	icp.loadModel("mesh.ply");

	while (true) {
		std::string numb;
		//cap.read(A);
		sequence >> colorImg;
		depthImg = loadDepth("data/depth" + std::to_string(counter) + ".dpt");
		//kin2.getKinectFrames(colorImg, depthImg);
		/*
		if (colorImg.empty() || counter == 250)
		{
			std::cout << "End of Sequence" << std::endl;
			std::cout << "Mean Diff: " <<accumDiff / counter<<std::endl;
			break;
		}
		*/


		inputImg.push_back(colorImg);
		inputImg.push_back(depthImg);
		line.detectTemplate(inputImg);

		std::vector<ObjectPose> detectedPoses;


		detectedPoses = line.getObjectPoses();
		
		//Model tmp;
		uint16 bestPose = 0;
		//ObjectPose groundTruth;
		//readGroundTruthLinemodDataset(counter, groundTruth);
		//opengl.readModelFile(modelFolder + modelFiles[0], tmp); //TODO welches file

		if (!detectedPoses.empty()) {
			///////////////////////////////////////////////////
			/*
			glm::vec3 eult = glm::eulerAngles(detectedPoses[0].quaternions);
			glm::qua quatst(glm::vec3(eult.x + M_PI / 2.0f, -eult.y, -eult.z));

			glm::mat4 newViewMatt = glm::toMat4(quatst);
			opengl.renderDepthToFrontBuff(&modBuff[0], newViewMatt, detectedPoses[0].translation); //TODO include translation in new view mat
			cv::Mat deptht = opengl.getDepthImgFromBuff();

			cv::Mat binaryt;
			
			depthToBinary(deptht, binaryt);
			*/
			///////////////////////////////////////////////////
			icp.prepareDepthForIcp(depthImg, cameraMatrix, detectedPoses[0].boundingBox);
			icp.registerToScene(detectedPoses);

			counter++;

			bestPose = icp.estimateBestMatch(depthImg, detectedPoses, opengl, modBuff[0]);
			drawCoordinateSystem(colorImg, cameraMatrix, 75.0f, detectedPoses[bestPose]);


			//float32 scoreNew = matchingScoreParallel(tmp, groundTruth, detectedPoses[bestPose]);
			//std::cout << "final " << bestPose << ": " << scoreNew << " : "<<bestMean<< std::endl;

			//if (scoreNew <= 14) {
			//	accumDiff++;
			//}
		}
		cv::imshow("color", colorImg);
		if (cv::waitKey(1) == 27) {
			break;
		}
		inputImg.clear();
	}


	std::getchar();
}
