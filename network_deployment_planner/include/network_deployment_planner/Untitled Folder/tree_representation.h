#ifndef TREE_REPRESENTATION_H_
#define TREE_REPRESENTATION_H_

#include <vector>
#include <map>
#include <set>
#include <string>


#include <algorithm>
#include <iostream>
#include <fstream>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>


struct point2d_
{
    double x,y;
    double angle;
    point2d_(){};
    point2d_(double x, double y){this->x = x; this->y =y;}
    point2d_(double x, double y, double angle){this->x = x; this->y =y; this->angle = angle;}
    void set(double x, double y){this->x = x; this->y =y;}
    double euclideanDistance(double x, double y){ return sqrt((x - this->x)*(x - this->x) + (y - this->y)*(y - this->y));}
    double euclideanDistance(point2d_ &point2){ return sqrt((point2.x - this->x)*(point2.x - this->x) + (point2.y - this->y)*(point2.y - this->y));}
    static double point2SegmentDistance(double x, double y, point2d_ &point1, point2d_ &point2);
    static point2d_ closestPoint2Segment(double x, double y, point2d_ &point1, point2d_ &point2);
    void operator=(point2d_ v1){this->x = v1.x; this->y = v1.y;}
};


namespace boost { namespace serialization {
        template<class Archive>
        void serialize(Archive & ar, point2d_& v, const unsigned int version)
        {
            ar & v.x; ar & v.y; ;
        }
    }
}


class TreeRepresentation{
private:
	int findClosestClient(double x, double y);
	bool findClosestSegment(double x, double y, int &first_node_index, int &second_node_index);
	bool recursiveSearch(int source_node_index, int first_destination_node_index, int second_destination_node_index, std::vector<int> &path);
	void add2Path(point2d_ point_a, point2d_ point_b, std::vector<point2d_> &path);

public:
	void setDistanceBetweenPoints(double distance_between_points);
	void findPath(double source_x, double source_y, double destination_x, double destination_y, std::vector<point2d_>& path);
	TreeRepresentation();
	
	void load(std::string filedir, std::string filename1, std::string filename2);

private:
	int* used_points_;
	int* used_points_index_;
	int* clients_;
	int num_points_;
	double distance_between_points_;
	std::map<int, std::vector<int>* > graph_;
	std::map<int, point2d_> vertices_;

};

#endif