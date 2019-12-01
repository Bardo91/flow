//---------------------------------------------------------------------------------------------------------------------
//  FLOW
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2019 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------

#include <flow/DataFlow.h>
#include <thread>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Eigen>

FLOW_TYPE_REGISTER("int", int)
FLOW_TYPE_REGISTER("float", float)
FLOW_TYPE_REGISTER("cloud", pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr)
FLOW_TYPE_REGISTER("mat44", Eigen::Matrix4f)
FLOW_TYPE_REGISTER("vec3", Eigen::Vector3f)
FLOW_TYPE_REGISTER("vec4", Eigen::Vector4f)
FLOW_TYPE_REGISTER("quat", Eigen::Quaternionf)

namespace flow{
    DataFlow::DataFlow(std::vector<std::pair<std::string, std::string>> _flows, std::function<void(DataFlow _f)> _callback){
        callback_ = _callback;
        for(auto &f:_flows){
            types_[f.first] = f.second;
            data_[f.first] = std::any();
            updated_[f.first] = false;
        }
    }

    void DataFlow::update(std::string _tag, std::any _data){
        if(data_.find(_tag)!= data_.end()){
            // Can we check here the type?
            data_[_tag] = _data;
            updated_[_tag] = true;
            checkData();
        }else{
            std::invalid_argument("Bad tag type while updating Dataflow");
        }
    }

    void DataFlow::checkData(){
        int flagCounter = 0;
        for(auto flag = updated_.begin(); flag != updated_.end(); flag++){
            if(flag->second) flagCounter++;
        }
        if(flagCounter == updated_.size()){
            // callback_(*this);
            std::thread(callback_, *this).detach(); // 666 Smthg is not completelly thread safe and produces crash
        }
    }
}