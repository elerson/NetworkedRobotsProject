#include "network_deployment_planner/tree_representation.h"
#include <cmath>
#include <limits>

#define PI 3.14159265

int TreeRepresentation::findClosestClient(double x, double y){

	std::map<int, point2d_>::iterator vertice_it;
	double distance = std::numeric_limits<double>::max();
	int client = -1;
	for(vertice_it = vertices_.begin(); vertice_it != vertices_.end(); vertice_it){
		int vertice = vertice_it->first;
		if(clients_[vertice]){// it is a client
			double vertices_distances = vertice_it->second.euclideanDistance(x, y);
			if( vertices_distances < distance){
				client = vertice;
				distance = vertices_distances;
			}
		}
	}
	return client;
}

bool TreeRepresentation::findClosestSegment(double x, double y, int &first_node_index, int &second_node_index){
	std::map<int, std::vector<int>* >::iterator fisrt_node_it;
	std::vector<int>::iterator second_node_it;
	double distance = std::numeric_limits<double>::max();
	first_node_index = -1;
	second_node_index = -1; 
	bool found_segment = false;
	for(fisrt_node_it = graph_.begin(); fisrt_node_it != graph_.end(); ++fisrt_node_it){
		std::vector<int>* second_node_vector = fisrt_node_it->second;
		for(second_node_it = second_node_vector->begin(); second_node_it != second_node_vector->end(); ++second_node_it){
			point2d_ first_node_position = vertices_[fisrt_node_it->first];
			point2d_ second_node_position = vertices_[*second_node_it];
			double calculate_distance = point2d_::point2SegmentDistance(x ,y, first_node_position, second_node_position);
			if( calculate_distance < distance){
				distance = calculate_distance;
				first_node_index = fisrt_node_it->first;
				second_node_index = *second_node_it;
				found_segment = true;
			}
		}
	}
	return found_segment;
}

void TreeRepresentation::findPath(double source_x, double source_y, double destination_x, double destination_y, std::vector<point2d_> &path){

	std::vector<int> index_path;	
	//find the client closest to destination
	int client_index = findClosestClient(destination_x, destination_y);

	//find the segment closest to the destination
	int first_source_closest_index, second_source_closest_index;
	bool source_closest_segment_found = findClosestSegment(source_x, source_y, first_source_closest_index, second_source_closest_index);

	//determine the path from the source to the fist point to the closest segment
	//this is done using a recursive function
	//then find the complementary path inside the closest segment
	recursiveSearch(client_index, first_source_closest_index, second_source_closest_index, index_path);
	//create the path where the robots will use as global planner
	for(int i = 0; i < index_path.size()-1 ; i++){
		point2d_ point_a, point_b;
		point_a = vertices_[index_path.at(i)];
		point_b = vertices_[index_path.at(i+1)];
		add2Path(point_a, point_b, path);
	}

	//the first point to the middle segment point is selected
	int first_point_index, second_point_index;
	if(first_source_closest_index != index_path.at(index_path.size()-1)){
		first_point_index = first_source_closest_index;
		second_point_index = second_source_closest_index;
	}else{
		first_point_index = second_source_closest_index;
		second_point_index = first_source_closest_index;
	}
	point2d_ fist_point = vertices_[first_point_index];	
	point2d_ second_point = vertices_[second_point_index];	

	//complete the remaining path
	point2d_ segment_point = point2d_::closestPoint2Segment(source_x ,source_y, fist_point, second_point);
	
	//create the path where the robots will use as global planner	
	add2Path(fist_point, segment_point, path);
	add2Path(segment_point, point2d_(destination_x, destination_y), path);
}

void TreeRepresentation::add2Path(point2d_ point_a, point2d_ point_b, std::vector<point2d_> &path){

		double distance = point_a.euclideanDistance(point_b);

		int number_of_segments = distance/distance_between_points_;

		double vec_x = point_a.x;
		double vec_y = point_a.y;

		double vec_delta_x = (point_b.x - point_a.x)/number_of_segments;
		double vec_delta_y = (point_b.y - point_a.y)/number_of_segments;
  
		double angle = atan2 (vec_delta_y,vec_delta_x); // 0 - 2pi
		angle = angle < 0 ? angle + 2*PI: angle ;

		for(int j = 0; j < number_of_segments; j++){
			
			path.push_back(point2d_(vec_x, vec_y, angle));
			vec_x +=vec_delta_x;
			vec_y +=vec_delta_y;
		}
}

void TreeRepresentation::setDistanceBetweenPoints(double distance_between_points){
	distance_between_points_ = distance_between_points;
}

bool TreeRepresentation::recursiveSearch(int source_node_index, int first_destination_node_index, int second_destination_node_index, std::vector<int> &path){
	
	if((source_node_index == first_destination_node_index) || (source_node_index == first_destination_node_index))
		return true;

	path.push_back(source_node_index);

	int new_source_node_index;
	std::vector<int>* vertices = graph_[source_node_index];
	for(int i = 0; i < vertices->size(); i++){
		new_source_node_index = vertices->at(i);
		if(recursiveSearch(new_source_node_index, first_destination_node_index, second_destination_node_index, path))
			return true;
	}
	path.pop_back();	
	
	return false;
}

void TreeRepresentation::load(std::string filedir, std::string filename1, std::string filename2 ){

	//create the tree filename full directory
    std::string fullfilename1 = filedir + filename1;
    std::string fullfilename2 = filedir + filename2;

    //create the file
    std::ifstream treefile1(fullfilename1.c_str(), std::ios::binary);
    std::ifstream treefile2(fullfilename2.c_str(), std::ios::binary);
    
    //open the graph file
    boost::archive::binary_iarchive inoputfile(treefile2);
    inoputfile >> graph_ >> vertices_;

    //first gets the num of points
    treefile1.read((char*)&num_points_,sizeof(int));
    used_points_ = new int[num_points_];
    used_points_index_ = new int[num_points_];
    clients_ = new int[num_points_];

    //then we take the other informations
    treefile1.read((char*)used_points_, num_points_*sizeof(int));
    treefile1.read((char*)used_points_index_, num_points_*sizeof(int));
    treefile1.read((char*)clients_, num_points_*sizeof(int));
    
    //close tree file
    treefile1.close();
    treefile2.close();
}
														/***AUXILIARY CODES**/
double point2d_::point2SegmentDistance(double x, double y, point2d_ &point1, point2d_ &point2){
	double a_x, a_y; // adjacent 
	a_x = point2.x - point1.x;
	a_y = point2.y - point1.y;

	double h_x, h_y; //
	h_x = x - point1.x;
	h_y = y - point1.y;
	// a . h scalar product
    double ah = a_x*h_x + a_y*h_y;
    // |a|
	double mod_a = sqrt(a_x*a_x + a_y*a_y);

	double mod_newa = ah/mod_a;
	// calculate the closest point in the segment to the point x,y
	point2d_ point_a;
	point_a.x = mod_newa*(a_x/mod_a);
	point_a.y = mod_newa*(a_y/mod_a);
	//calculate
	double distance = sqrt((point_a.x-x)*(point_a.x-x) + (point_a.y-y)*(point_a.y-y));
	return distance;

}

static point2d_ closestPoint2Segment(double x, double y, point2d_ &point1, point2d_ &point2){

		double a_x, a_y; // adjacent 
	a_x = point2.x - point1.x;
	a_y = point2.y - point1.y;

	double h_x, h_y; //
	h_x = x - point1.x;
	h_y = y - point1.y;
	// a . h scalar product
    double ah = a_x*h_x + a_y*h_y;
    // |a|
	double mod_a = sqrt(a_x*a_x + a_y*a_y);

	double mod_newa = ah/mod_a;
	// calculate the closest point in the segment to the point x,y
	point2d_ point_a;
	point_a.x = mod_newa*(a_x/mod_a);
	point_a.y = mod_newa*(a_y/mod_a);
	return point_a;
}