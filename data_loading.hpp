#include <map>
#include <fstream>
#include <string>
#include <optional>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <eigen3/Eigen/Dense>

#include "csv_iterator.hpp"

typedef unsigned long long ullong;

//plugin for reading VCU-RVI dataset
class lazy_load_image {
public:
	lazy_load_image(const std::string& path)
		: _m_path(path)
	{ }
	std::unique_ptr<cv::Mat> unmodified_load() const {
		auto img = std::unique_ptr<cv::Mat>{new cv::Mat{cv::imread(_m_path, cv::IMREAD_UNCHANGED)}};
		assert(!img->empty());
		return img;
	}
	std::unique_ptr<cv::Mat> modified_load() const {
		//auto img = std::unique_ptr<cv::Mat>{new cv::Mat{cv::imread(_m_path, cv::IMREAD_UNCHANGED)}};
		cv::Mat original_mat = cv::imread(_m_path, cv::IMREAD_UNCHANGED);
		cv::Mat *converted_mat = new cv::Mat();
		cv::cvtColor(original_mat,*converted_mat,cv::COLOR_GRAY2RGBA,0);
		converted_mat->convertTo(*converted_mat, CV_8UC4);
		auto img = std::unique_ptr<cv::Mat>{converted_mat};
		assert(!img->empty());
		return img;
	}
    const std::string get_path()
    {
        return _m_path;
    }
private:
	std::string _m_path;
};

typedef struct {
	Eigen::Vector3f position;
	Eigen::Quaternionf orientation;
	std::optional<lazy_load_image> cam0;
	std::optional<lazy_load_image> cam1;
} sensor_types;

static
std::map<ullong, sensor_types>

load_data() {
	const char* illixr_data_c_str = std::getenv("ILLIXR_DATA");
	if (!illixr_data_c_str) {
		std::cerr << "Please define ILLIXR_DATA" << std::endl;
		abort();
	}
	std::string illixr_data = std::string{illixr_data_c_str};

	std::map<ullong, sensor_types> data;
	
    //pyh hardcode datapath inside the user provided data variable
    const std::string dataset_subpath = "/file.csv";

	std::cout<< "loading pose file : "<< illixr_data<<dataset_subpath <<"\n";
	std::ifstream dataset_file {illixr_data + dataset_subpath};
	if (!dataset_file.good()) {
		std::cerr << "${ILLIXR_DATA}" << dataset_subpath << " (" << illixr_data << dataset_subpath << ") is not a good path" << std::endl;
		abort();
	}
	std::string unit_conv;
	//not skipping the first line 
	for(CSVIterator row{dataset_file, 0}; row != CSVIterator{}; ++row) {
	    unit_conv = row[0].c_str();
        //pyh to fit data limit 
        auto decimal_start = unit_conv.find(".");
        auto decimal_digits = unit_conv.size() - decimal_start -1; 
        if(decimal_start != std::string::npos) {unit_conv.erase(decimal_start,1);}
        //for vcu comment out the following line, for tum uncomment
        //unit_conv.erase(0,3);
        ullong t = std::stoull(unit_conv);
        //std::cout<<"pre t: "<<t<<std::endl;
        t = t * pow(10,(9-decimal_digits));
        //std::cout<<"depth t: "<<t<<std::endl;
        //these are used for pose input
		data[t].cam0 = {illixr_data +  "/" + row[9]};
		//pyh enable color
		data[t].cam1 = {illixr_data +  "/" + row[11]};
		////tx ty tz
		data[t].position(0) = std::stof(row[1]);
		//std::cout<<data[t].position(0)<<std::endl;
		data[t].position(1) = std::stof(row[2]);
		data[t].position(2) = std::stof(row[3]);
        ////qx qy qz qw
		data[t].orientation.x() = std::stof(row[4]);
		data[t].orientation.y() = std::stof(row[5]);
		data[t].orientation.z() = std::stof(row[6]);
		data[t].orientation.w() = std::stof(row[7]);
//		std::cout<<"data loading orientation: \n"<<data[t].orientation.w()<<std::endl;


        //these are used for transformation matrix input
		//data[t].cam0 = {illixr_data +  "/" + row[14]};
		//data[t].cam1 = {illixr_data +  "/" + row[16]};
		////std::cout<<illixr_data <<  "/" <<row[14]<<"\n";
		////std::cout<<illixr_data <<  "/" <<row[16]<<"\n";
		//data[t].transformation(0,0) = std::stof(row[1]);
		//data[t].transformation(1,0) = std::stof(row[2]);
		//data[t].transformation(2,0) = std::stof(row[3]);
		//data[t].transformation(3,0) = 0.0f;
		//data[t].transformation(0,1) = std::stof(row[4]);
		//data[t].transformation(1,1) = std::stof(row[5]);
		//data[t].transformation(2,1) = std::stof(row[6]);
		//data[t].transformation(3,1) = 0.0f;
		//data[t].transformation(0,2) = std::stof(row[7]);
		//data[t].transformation(1,2) = std::stof(row[8]);
		//data[t].transformation(2,2) = std::stof(row[9]);
		//data[t].transformation(3,2) = 0.0f;
		//data[t].transformation(0,3) = std::stof(row[10]);
		//data[t].transformation(1,3) = std::stof(row[11]);
		//data[t].transformation(2,3) = std::stof(row[12]);
		//data[t].transformation(3,3) = 1.0f;
        
	}
	std::cout<<"finished data loading size: "<<data.size()<<std::endl;
	return data;
}
