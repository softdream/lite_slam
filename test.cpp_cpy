#include "occupied_grid_map.h"

#include "laserSimulation.h"

#include <opencv2/opencv.hpp>


void laserData2Container( const sensor::LaserScan &scan, sensor::ScanContainer &container )
{
        size_t size = 1440;

        float angle = -3.14159f;
        container.clear();

        for( int i = 0; i < size; i ++ ){
                float dist = scan.ranges[ i ];

                if( dist >= 0.0099999998f && dist <= 14.0000000000f ){
                        //dist *= scaleToMap;
                        container.addData( Eigen::Vector2f( cos(angle) * dist, sin(angle) * dist ) );
                }

                angle += 0.0043633231f;
        }

        std::cout<<"Scan Container Size: "<<container.getSize()<<std::endl;
}


int main()
{
	std::cout<<"------------- Occupied Grid Map Test --------------"<<std::endl;

	grid::OccupiedGridMap<float> occupied_map;
	occupied_map.printMapInfo();

	cv::Mat image = cv::Mat::zeros(occupied_map.getSizeX(), occupied_map.getSizeY(), CV_8UC3);


	simulation::Simulation simulation;
	simulation.openSimulationFile( "frame1.txt" );
	
	sensor::LaserScan scan;
	simulation.readAFrameData( scan );

	sensor::ScanContainer container;
	laserData2Container( scan, container );

	Eigen::Vector3f robot_pose( 0.0, 0.0, 0.0 );
	occupied_map.updateMapByScan( container, robot_pose );
	
	std::cout<<"---------------- Get the Map -----------------"<<std::endl;

        int occupiedCount = 0;
	for( int i = 0; i < occupied_map.getSizeX(); i ++ ){
                for( int j = 0; j < occupied_map.getSizeY(); j ++ ){
			if( occupied_map.isCellFree( i, j ) ){
                                cv::circle(image, cv::Point2d(i, j), 1, cv::Scalar(255, 255, 255), 1);
			}
			else if( occupied_map.isCellOccupied( i, j ) ){
                                occupiedCount ++;
                                cv::circle(image, cv::Point2d(i, j), 1, cv::Scalar(0, 0, 255), 1);
                        }
		}
	}

	std::cout<<"---------------- Result --------------------"<<std::endl;
        std::cout<<"Occupied Points Number: "<<occupiedCount<<std::endl;

	cv::imshow( "test", image );

        cv::waitKey(0);
	
	return 0;
}
