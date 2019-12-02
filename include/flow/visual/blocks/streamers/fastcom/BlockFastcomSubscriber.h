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

#ifndef FLOW_BLOCKS_STREAMERS_FASTCOM_FASTCOMSUBSCRIBER_H_
#define FLOW_BLOCKS_STREAMERS_FASTCOM_FASTCOMSUBSCRIBER_H_

#include <flow/Block.h>
#include <flow/Outpipe.h>

#include <fastcom/fastcom.h>
#include <opencv2/opencv.hpp>

namespace flow{
	template<typename _Trait >
    class BlockFastcomSubscriber : public flow::Block{
    public:
		BlockFastcomSubscriber(){
            for (auto tag : _Trait::output_)
		        opipes_[tag.first] = new flow::Outpipe(tag.first, tag.second);
			}

        static std::string name() { 
			return _Trait::blockName_;
		}
	
        virtual bool configure(std::unordered_map<std::string, std::string> _params) override{
			
			std::string ipAdress = _params["ip"];
			int portNumber = std::stoi(_params["port"]);
			
			sub_ = fastcom::Subscriber<typename _Trait::DataType_>(ipAdress , portNumber);

			sub_->attachCallback(subsCallback);
	    	return true;
	    }

        std::vector<std::string> parameters() override {return {"ip" , "port"};} 

    private:
        void subsCallback(typename _Trait::DataType_ &_msg){
			for (auto tag : _Trait::output_){
				if(opipes_[tag.first]->registrations() !=0 ){
               		opipes_[tag.first]->flush(_msg);
				}
			}
        }

    private:
		fastcom::Subscriber<typename _Trait::DataType_> sub_;
    };

    struct TraitFastcomImageSubscriber{
        static std::string blockName_;
	    static std::pair<std::string, std::string> input_;
	    typedef cv::Mat DataType_;
    }; 

	std::string TraitFastcomImageSubscriber::blockName_ = "Fastcom Subscriber image";
	std::pair<std::string, std::string> TraitFastcomImageSubscriber::input_ = std::make_pair("Image", "image");

    typedef BlockFastcomSubscriber< TraitFastcomImageSubscriber > BlockFastcomImageSubscriber;

}

#endif
