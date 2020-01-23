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

#include <opencv2/opencv.hpp>
#ifdef FLOW_USE_FASTCOM
	#include <fastcom/fastcom.h>
#endif

namespace flow{
    template<typename _Trait >
    class BlockFastcomPublisher : public flow::Block{
    public:
        static std::string name() {return _Trait::blockName_; }
        std::string description() const override {return    "Communication block using fastcom.\n"+
                                                            "Publisher actor that sends data of type "+ _Trait::input_ +".";};

		BlockFastcomPublisher(){

            createPolicy({_Trait::input_});
            for (auto tag : _Trait::input_){
                registerCallback({tag.first}, 
                                        [&](DataFlow _data){
                                            auto data = _data.get<typename _Trait::DataType_>(tag.first);
                                            #ifdef FLOW_USE_FASTCOM
                                                pub_->publish(data);
                                            #endif  
                                        }
                );
            }        
        };

        virtual bool configure(std::unordered_map<std::string, std::string> _params) override{
            
            int portNumber = std::stoi(_params["port"]);
            #ifdef FLOW_USE_FASTCOM
                pub_ = new fastcom::ImagePublisher(portNumber);
                return true;
            #else
                return false;
            #endif
			
        }
        std::vector<std::string> parameters() override {return {"port"};}

    private:
        #ifdef FLOW_USE_FASTCOM
            fastcom::ImagePublisher *pub_; // 666
        #endif
    };

    struct TraitFastcomImagePublisher{
        static std::string blockName_;
	    static std::map<std::string, std::string> input_;
	    typedef cv::Mat DataType_;
    }; 

	std::string TraitFastcomImagePublisher::blockName_ = "Fastcom Publisher image";
	std::map<std::string, std::string> TraitFastcomImagePublisher::input_ = {{{"Color Image", "image"}}};

    typedef BlockFastcomPublisher< TraitFastcomImagePublisher > BlockFastcomImagePublisher;

}

#endif