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

#include <flow/Block.h>
#include <flow/Outpipe.h>

#include <cassert>

namespace flow{
    // BASE METHODS


    std::unordered_map<std::string, Outpipe*> Block::getPipes(){
        return opipes_;
    }

    void Block::start(){
        runLoop_ = true;
        loopThread_ = std::thread(&Block::loopCallback, this);
    }

    void Block::stop(){
        runLoop_ = false;
        if(loopThread_.joinable())
            loopThread_.joinable();
    }

    
    int Block::nInputs(){
        if(iPolicy_)
            return iPolicy_->nInputs();
        else
            return 0;
    }

    std::vector<std::string> Block::inputTags(){
        if(iPolicy_)
            return iPolicy_->inputTags();
        else
            return {};
    }

    int Block::nOutputs(){
        return opipes_.size();
    }

    std::vector<std::string> Block::outputTags(){
        std::vector<std::string> tags;
        for(auto &os: opipes_){
            tags.push_back(os.first);
        }

        return tags;
    }

    Policy* Block::getPolicy(){
        return iPolicy_;
    }

    void Block::connect(std::string _pipeTag, Block &_otherBlock){
        // if(opipes_[_pipeTag] != nullptr){
        //     opipes_[_pipeTag]->registerPolicy(_otherBlock.getPolicy());
        // }
    }

    void Block::disconnect(std::string _pipeTag) {
        iPolicy_->disconnect(_pipeTag);
    }

}