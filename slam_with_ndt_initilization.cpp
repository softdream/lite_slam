#include <iostream>
#include "slam_process.h"

#include "laserSimulation.h"

#include <opencv2/opencv.hpp>

#include <chrono>

#include <map>

#include <fstream>

#include "map_manage.h"

#include "scan_context_pca.h"

#include "ndt_grid.h"

void laserData2Container( const sensor::LaserScan &scan, sensor::ScanContainer &container )
{
        size_t size = 1440;

        float angle = -3.12413907051f;
        container.clear();

        for( int i = 0; i < size; i ++ ){
                float dist = scan.ranges[ i ];

                if( dist >= 0.0099999998f && dist <= 8.0000000000f ){
                        //dist *= scaleToMap;
                        container.addData( Eigen::Vector2f( cos(angle) * dist, sin(angle) * dist ) );
                }

                angle += 0.00435422640294f;
        }

        std::cout<<"Scan Container Size: "<<container.getSize()<<std::endl;
}



int main()
{
	std::cout<<" --------------- SLAM TEST --------------"<<std::endl;

	slam::SlamProcessor<float> slam;
	scancontext::ScanContextPCA<float> scan_context;
	
	ndt::NDT ndt;

	// print the map information
	slam.printMapInfo();
	cv::Mat image = cv::Mat::zeros(slam.getSizeX(), slam.getSizeY(), CV_8UC3);
	cv::imshow("map", image);

	simulation::Simulation simulation;
	simulation.openSimulationFile( "../../test_data/laser_data.txt" );

	Eigen::Vector3f robot_pose( 0.0f, 0.0f, 0.0f );
	std::vector<Eigen::Vector3f> key_poses;
	std::vector<Eigen::Vector3f> loop_poses_old;
	std::vector<Eigen::Vector3f> loop_poses_new;

	sensor::ScanContainer pre_scan_container;
	while( !simulation.endOfFile() ){
		sensor::LaserScan scan;
		sensor::ScanContainer scan_container;
	
		simulation.readAFrameData( scan );
		laserData2Container( scan, scan_container );
	
		std::cout<<"frame count: "<<simulation.getFrameCount()<<std::endl;

		if( simulation.getFrameCount() <= 10  ){
			slam.processTheFirstScan( robot_pose, scan_container );
			if( simulation.getFrameCount() == 10 ){
				slam.displayMap( image );
				
				key_poses.push_back( robot_pose );
				pre_scan_container = scan_container;
			}
		}
		else {
			auto beforeTime = std::chrono::steady_clock::now();
			
			// added for ndt
			Eigen::Vector3f initilized_delta( 0.0, 0.0, 0.0 );
                        ndt.ndtProcess( pre_scan_container, scan_container, initilized_delta, 3 );
			std::cout<<"delta: "<<initilized_delta.norm()<<std::endl<<initilized_delta<<std::endl;
			if( initilized_delta.norm() < 0.1 ){
				robot_pose += initilized_delta;
				std::cout<<"initilized robot pose: "<<std::endl<<robot_pose<<std::endl;
			}
			auto afterTime = std::chrono::steady_clock::now();
			double duration_millsecond = std::chrono::duration<double, std::milli>(afterTime - beforeTime).count();
			std::cout<<"ndt duration : " << duration_millsecond << "ms" << std::endl;
			// for scan to map
			beforeTime = std::chrono::steady_clock::now();
			slam.update( robot_pose, scan_container );
			robot_pose = slam.getLastScanMatchPose();
			afterTime = std::chrono::steady_clock::now();
			duration_millsecond = std::chrono::duration<double, std::milli>(afterTime - beforeTime).count();
			std::cout<<"scan to map duration : " << duration_millsecond << "ms" << std::endl;
			
			std::cout<<"robot pose now: "<<std::endl;
                        std::cout<<robot_pose<<std::endl;
                        std::cout<<"------------------"<<std::endl;

			if( slam.isKeyFrame() ){
				key_poses.push_back( robot_pose );

				slam.displayMap( image );
			}

			pre_scan_container = scan_container;
		}
		
		
		cv::waitKey(5);
	}

	//map::MapManagement<float>::saveOccupiedGridMap( "test.map", slam.getOccupiedGridMap() );

	cv::waitKey(0);
	simulation.closeSimulationFile();

	return 0;
}
