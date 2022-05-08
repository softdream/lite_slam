#include "localization.h"

void laserData2Container( const sensor::LaserScan &scan, sensor::ScanContainer &container )
{
	size_t size = 1440;

        float angle = -3.14159f;
        container.clear();

        for( int i = 0; i < size; i ++ ){
        	float dist = scan.ranges[ i ];

                if( dist >= 0.0099999998f && dist <= 8.0000000000f ){
                	container.addData( Eigen::Vector2f( cos(angle) * dist, sin(angle) * dist ) );
               	}

             	angle += 0.0043633231f;
  	}

        std::cout<<"Scan Container Size: "<<container.getSize()<<std::endl;
}

int main()
{
	std::cout<<"----------------- LOCALIZATION TEST ---------------"<<std::endl;
	std::ofstream out("pose.txt", std::ios::app);

	localization::Localization<float> locate( "../../test_data/test.map" );

	simulation::Simulation simulation;
        simulation.openSimulationFile( "../../test_data/localization_laser_data_1080.txt" );

	Eigen::Vector3f robot_pose( 0.0, 0.0, 0.0 );
	while( !simulation.endOfFile() ){
		sensor::LaserScan scan;	
		simulation.readAFrameData( scan ); // read the laser data
		std::cout<<"frame count: "<<simulation.getFrameCount()<<std::endl;		

		if( !locate.isPoseInitialized() ){
			robot_pose = locate.getInitializationPose( "../../test_data/key_scans.txt", "../../test_data/key_poses.txt", scan );
	
			std::cout<<"initialized pose = "<<robot_pose<<std::endl;
		}
		else {
			sensor::ScanContainer container;
			locate.laserData2Container( scan, container );
			locate.update( robot_pose, container );
			robot_pose = locate.getLastScanMatchPose();
			std::cout<<"robot pose: "<<std::endl<<robot_pose<<std::endl;
			out << robot_pose[0]<<" "<<robot_pose[1]<<" "<<robot_pose[2]<<std::endl;
		}
	
	}
	
	out.close();
	return 0;
}
