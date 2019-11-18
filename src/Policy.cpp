//---------------------------------------------------------------------------------------------------------------------
//  FLOW
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
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

#include <flow/Policy.h>

#include <flow/OutPipe.h>

#include <cassert>
#include <stdexcept>


namespace flow{

    Policy::Policy(std::vector<std::string> _inPipes){
        if(_inPipes.size() == 0){
            throw std::invalid_argument( "A Policy cannot be constructed with an empty list of input pipe tags." );
        }

        tags_ = _inPipes;
        for(auto &tag: _inPipes){
            dataFlow_[tag] = std::any();
            validData_[tag] = false;
        }
    }

    bool Policy::registerCallback(PolicyMask _mask, PolicyCallback _callback){
        int existingTags = 0;
        for(auto t0: _mask){
            auto iter = std::find(tags_.begin(), tags_.end(), t0);
            if(iter != tags_.end()){
                existingTags++;
            }
        }
        
        if(existingTags == _mask.size()){    // All tags are in the policy
            callbacks_.push_back({_mask, _callback});
            return true;
        }else{
            return false;
        }

    }

    void Policy::update(std::string _tag, std::any _val){
        dataFlow_[_tag] = _val;
        validData_[_tag] = true;
        checkMasks();
    }

    int Policy::nInputs(){
        return tags_.size();
    }

    std::vector<std::string> Policy::inputTags(){
        return tags_;
    }

    void Policy::associatePipe(std::string _tag, OutPipe* _pipe){
        connetedPipes_[_tag] = _pipe;
    }

    void Policy::disconnect(std::string _tag){
        connetedPipes_[_tag]->unregisterPolicy(this);
    }

    void Policy::checkMasks(){
        for(auto &pairCb: callbacks_){  // Check all pairs mask-cb
            auto maskTags = pairCb.first;
            unsigned counter = 0;
            for(auto &tag: maskTags){   // Check all tags in mask
                for(auto iter = validData_.begin(); iter != validData_.end(); iter++){
                    if(iter->first == tag && validData_[tag]){
                        counter++;
                        break;
                    }
                }
            }
            if(counter ==  maskTags.size()){
                for(auto&tag:maskTags){ // uff... For more complex pipelines with shared data might not work... need conditions per callback.
                    validData_[tag] = false;
                }
                std::thread(pairCb.second, dataFlow_).detach(); // 666 Smthg is not completelly thread safe and produces crash
                // pairCb.second(dataFlow_);
            }
        }
    }

}