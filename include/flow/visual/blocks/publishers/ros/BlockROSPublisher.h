//---------------------------------------------------------------------------------------------------------------------
//  flow
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

#ifndef FLOW_BLOCKS_PUBLISHERS_ROS_BLOCKROSPUBLISHER_H_
#define FLOW_BLOCKS_PUBLISHERS_ROS_BLOCKROSPUBLISHER_H_

#include <flow/Block.h>
#include <flow/Outpipe.h>

#ifdef FLOW_USE_ROS
	#include <ros/ros.h>
#endif

namespace flow{

    template<typename _Trait >
    class BlockROSPublisher : public flow::Block{
    public:
        static std::string name() {return _Trait::blockName_; }

		BlockROSPublisher(){

            iPolicy_ = new flow::Policy({_Trait::input_});

            iPolicy_->registerCallback({_Trait::input_.first}, 
                                    [&](DataFlow _data){
                                        typename _Trait::ROSType_ topicContent = _Trait::conversion_(_data);
                                        #ifdef FLOW_USE_ROS
                                            pubROS_.publish(topicContent);
                                        #endif
                                    }
            );
        };

        virtual bool configure(std::unordered_map<std::string, std::string> _params) override{
            #ifdef FLOW_USE_ROS
                std::string topicPublish = _params["topic"];
                pubROS_ = nh_.advertise< typename _Trait::ROSType_ >(topicPublish, 1 );
			#endif
            return true;
        }
        std::vector<std::string> parameters() override {return {"topic"};}

    private:
		#ifdef FLOW_USE_ROS
			ros::NodeHandle nh_;
			ros::Publisher pubROS_;
		#endif
    };
}


#endif
