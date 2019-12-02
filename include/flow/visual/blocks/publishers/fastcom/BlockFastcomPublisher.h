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

#ifndef FLOW_BLOCKS_PUBLISHERS_FASTCOM_BLOCKFASTCOMPUBLISHERS_H_
#define FLOW_BLOCKS_PUBLISHERS_FASTCOM_BLOCKFASTCOMPUBLISHERS_H_

#include <fastcom/fastcom.h>
#include <opencv2/opencv.hpp>

namespace flow{
    template<typename _Trait >
    class BlockFastcomPublisher : public flow::Block{
    public:
        static std::string name() {return _Trait::blockName_; }

		BlockFastcomPublisher(){

            iPolicy_ = new flow::Policy({_Trait::input_});

            iPolicy_->registerCallback({_Trait::input_.first}, 
                                    [&](DataFlow _data){
                                        auto data = _data.get<typename _Trait::DataType_>(_Trait::input_.first);
                                        pub_.publish(data);  
                                    }
            );
        };

        virtual bool configure(std::unordered_map<std::string, std::string> _params) override{
            
                int portNumber = std::stoi(_params["port"]);
                pub_ = fastcom::Publisher<typename _Trait::DataType_>(portNumber);
			
            return true;
        }
        std::vector<std::string> parameters() override {return {"port"};}

    private:
        fastcom::Publisher<typename _Trait::DataType_> pub_;
    };

    struct TraitFastcomImagePublisher{
        static std::string blockName_;
	    static std::pair<std::string, std::string> input_;
	    typedef cv::Mat DataType_;
    }; 

	std::string TraitFastcomImagePublisher::blockName_ = "Fastcom Publisher image";
	std::pair<std::string, std::string> TraitFastcomImagePublisher::input_ = std::make_pair("Image", "image");

    typedef BlockFastcomPublisher< TraitFastcomImagePublisher > BlockFastcomImagePublisher;

}

#endif