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
#include <opencv2/opencv.hpp>
#ifdef FLOW_USE_FASTCOM
	#include <fastcom/fastcom.h>
#endif

namespace flow{
	template<typename _Trait >
    class BlockFastcomSubscriber : public flow::Block{
    public:
		BlockFastcomSubscriber(){
            for (auto tag : _Trait::output_)
		        createPipe(tag.first, tag.second);
			}

        static std::string name() { 
			return _Trait::blockName_;
		}
	
        virtual bool configure(std::unordered_map<std::string, std::string> _params) override{
			
			std::string ipAdress = _params["ip"];
			int portNumber = std::stoi(_params["port"]);
			#ifdef FLOW_USE_FASTCOM
				sub_ = new fastcom::ImageSubscriber(ipAdress , portNumber);
				sub_->attachCallback(std::bind(&subsCallback, this, _1));
				
	    		return true;
			#else
				return false;
			#endif
	    }

        std::vector<std::string> parameters() override {return {"ip" , "port"};} 

    private:
        void subsCallback(typename _Trait::DataType_ &_msg){
			for (auto tag : _Trait::output_){
				if(getPipe(tag.first)->registrations() !=0 ){
               		getPipe(tag.first)->flush(_msg);
				}
			}
        }

    private:
		#ifdef FLOW_USE_FASTCOM
			std::shared_ptr<fastcom::ImageSubscriber> sub_;
		#endif
    };

    struct TraitFastcomImageSubscriber{
        static std::string blockName_;
	    static std::map<std::string, std::string> output_;
	    typedef cv::Mat DataType_;
    }; 

	std::string TraitFastcomImageSubscriber::blockName_ = "Fastcom Subscriber image";
	std::map<std::string, std::string> TraitFastcomImageSubscriber::output_ = {{{"Color Image", "image"}}};

    typedef BlockFastcomSubscriber< TraitFastcomImageSubscriber > BlockFastcomImageSubscriber;

}

#endif
