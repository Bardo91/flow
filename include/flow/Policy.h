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


#ifndef FLOW_POLICY_H_
#define FLOW_POLICY_H_

#include <vector>
#include <cstdlib>

#include <any>
#include <unordered_map>
#include <thread>
#include <chrono>
#include <iostream>
#include <functional>

namespace flow{
    class OutPipe;

    class Policy{
        public:
            typedef std::vector<std::string> PolicyMask;
            typedef std::function<void(std::unordered_map<std::string,std::any>)> PolicyCallback;

            Policy(std::vector<std::string> _inPipes);

            bool registerCallback(PolicyMask _mask, PolicyCallback _callback);

            void update(std::string _tag, std::any _val);
    
            int nInputs();
            std::vector<std::string> inputTags();

            void associatePipe(std::string _tag, OutPipe* _pipe);

            void disconnect(std::string _tag);
        private:
            void checkMasks();

        private:
            std::unordered_map<std::string, std::any>   dataFlow_;
            std::unordered_map<std::string, bool>       validData_; 
            std::unordered_map<std::string, OutPipe*>   connetedPipes_; 
            std::vector<std::string>                    tags_;
            std::vector<std::pair<PolicyMask, PolicyCallback>> callbacks_;
    };
}



#endif