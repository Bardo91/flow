//---------------------------------------------------------------------------------------------------------------------
//  mico
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

#ifndef FLOW_BLOCKS_STREAMERS_ROS_ROSSUBSCRIBER_H_
#define FLOW_BLOCKS_STREAMERS_ROS_ROSSUBSCRIBER_H_

#include <flow/Block.h>
#include <flow/Outpipe.h>

#ifdef FLOW_USE_ROS
	#include <ros/ros.h>
#endif

namespace flow{
	template<typename _Trait >
    class BlockROSSubscriber : public flow::Block{
    public:
		BlockROSSubscriber(){
            for (auto tag : _Trait::output_)
		        opipes_[tag.first] = new flow::Outpipe(tag.first, tag.second);
			}
		
		// ~BlockROSSubscriber(){};

        static std::string name() { 
			return _Trait::blockName_;
		}

        virtual bool configure(std::unordered_map<std::string, std::string> _params) override{
			#ifdef FLOW_USE_ROS
            	subROS_ = nh_.subscribe<typename _Trait::ROSType_>(_params["topic"], 1 , &BlockROSSubscriber::subsCallback, this);
			#endif
	    	return true;
	    }

        std::vector<std::string> parameters() override {return {"topic"};} 

    private:
        void subsCallback(const typename _Trait::ROSType_::ConstPtr &_msg){
			for (auto tag : _Trait::output_){
				if(opipes_[tag.first]->registrations() !=0 ){
               		opipes_[tag.first]->flush(_Trait::conversion_(tag.first , _msg));
				}
			}
        }

    private:
		#ifdef FLOW_USE_ROS
			ros::NodeHandle nh_;
			ros::Subscriber subROS_;
		#endif
    };

}


#endif
