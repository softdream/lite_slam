#ifndef __LOCALIZATION_H
#define __LOCALIZATION_H

#include <algorithm>

#include "laserSimulation.h"
#include "scan_match.h"
#include "ndt_grid.h"
#include "map_manage.h"

namespace localization
{

template<typename T>
class Localization
{
public:
	using DataType = T;
	
	Localization() = delete;

	Localization( const std::string &file_name )
	{
		grid_map_ = new grid::OccupiedGridMap<T>();
                if( grid_map_ == nullptr ){
                        std::cerr<<"Construct Occupied Grid Map Failed !"<<std::endl;
                        exit( -1 );
                }
                std::cerr<<"Construct The Occupied Grid Map !"<<std::endl;

                scan_match_ = new match::ScanMatchMethod<T>();
                if( scan_match_ == nullptr ){
                        std::cerr<<"Construct Scan Match Method Failed !"<<std::endl;
                        exit(-1);
                }
                std::cerr<<"Construct The Scan Match Method !"<<std::endl;

		*grid_map_ = map::MapManagement<float>::loadOccupiedGridMap( file_name );
	}

	~Localization()
	{
		if( grid_map_ != nullptr )
                        delete grid_map_;
                if( scan_match_ != nullptr )
                        delete scan_match_;
	}

	void update( const Eigen::Matrix<DataType, 3, 1> &robot_pose_in_world,
                     const sensor::ScanContainer &scan,
		     bool map_without_matching = false )
	{
		Eigen::Matrix<DataType, 3, 1> new_pose_estimated;
	
		if( !map_without_matching ){
                        new_pose_estimated = scan_match_->scanToMap( *grid_map_, robot_pose_in_world, scan, covarince_matrix_, 20 );
                }
                else {
                        new_pose_estimated = robot_pose_in_world;
                }
		
		last_scan_match_pose_ = new_pose_estimated;
	}

	const Eigen::Matrix<DataType, 3, 1>& getLastScanMatchPose() const
        {
                return last_scan_match_pose_;
        }

	const Eigen::Matrix<DataType, 3, 1> getInitializationPose( const std::string &key_scan_file_name, const std::string &key_pose_file_name, const sensor::LaserScan &curr_scan ) const
	{
		std::vector<sensor::LaserScan> candidate_scans;
		std::vector<Eigen::Matrix<DataType, 3, 1>> candidate_poses;
	
		simulation::Simulation scan_read;
		scan_read.openSimulationFile( key_scan_file_name );		
		
		while( !scan_read.endOfFile() ){
			sensor::LaserScan scan;
			scan_read.readAFrameData( scan );
			candidate_scans.push_back( scan );
		}

		std::ifstream odom_read( key_pose_file_name, std::ifstream::in );	
		if( !odom_read.is_open() ){
			std::cerr<<"Failed to open key poses file !"<<std::endl;
			exit(-1);
		}
	
		while( !odom_read.eof() ){
			std::string line;
                	std::getline( odom_read, line );
			std::istringstream iss( line );
                	std::string num;
			
			Eigen::Matrix<DataType, 3, 1> pose;
			iss >> num;	
			if( num.compare( "odom" ) == 0 ){
				for( int i = 0; i < 3; i ++ ){
					iss >> num;	
					pose[i] = stof( num );
				}
			}	
			candidate_poses.push_back( pose );
		}

		if( candidate_poses.size() != candidate_scans.size() ){
			std::cerr<<"Files are wrong !"<<std::endl;
			exit(-1);
		}

		ndt::NdtGrid<DataType> ndt;
		std::vector<DataType> scores;
		std::vector<Eigen::Matrix<DataType, 3, 1>> deltas;

		sensor::ScanContainer required_container;
        	laserData2Container( curr_scan, required_container );
		for( int i = 0; i < candidate_poses.size(); i ++ ){
			sensor::ScanContainer candidate_container;
                	laserData2Container( candidate_poses[i], candidate_container );
			Eigen::Matrix<DataType, 3, 1> p( 0.0, 0.0, 0.0 );
			ndt.ndtProcess( required_container, candidate_container, p );
			DataType score = ndt.getMatchScore();
			scores.push_back( score );
			deltas.push_back( deltas );
		}
		
		int position = -1;
		auto it = std::min_element( scores.begin(), scores.end() );
		if( it != scores.end() ){
			std::cout <<"min score : "<<*it<<std::endl;
			position = it - scores.begin();
			std::cout<<"vector pose : "<<position <<std::endl;
		}
		
		Eigen::Matrix<DataType, 3, 1> init_pose = candidate_poses[position] - deltas[position];

		is_initialized_ = true;	
	
		return init_pose;
	}

	void laserData2Container( const sensor::LaserScan &scan, sensor::ScanContainer &container )
{
        size_t size = 1440;

        float angle = -3.14159f;
        container.clear();

        for( int i = 0; i < size; i ++ ){ 
                float dist = scan.ranges[ i ];

                if( dist >= 0.0099999998f && dist <= 8.0000000000f ){
                        //dist *= scaleToMap;
                        container.addData( Eigen::Vector2f( cos(angle) * dist, sin(angle) * dist ) );
                }

                angle += 0.0043633231f;
        }

        std::cout<<"Scan Container Size: "<<container.getSize()<<std::endl;
}

	bool isPoseInitialized() const
	{
		return is_initialized_;
	}

private:
        match::ScanMatchMethod<T> *scan_match_;
        grid::OccupiedGridMap<T> *grid_map_;

	Eigen::Matrix<DataType, 3, 3> covarince_matrix_;

	Eigen::Matrix<DataType, 3, 1> last_scan_match_pose_;
	
	bool is_initialized_ = false;
};

}


#endif
