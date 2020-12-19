//---------------------------------------------------------------------------------------------------------------------
//  FLOW
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2020 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
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


#ifndef FLOW_BLOCK_H_
#define FLOW_BLOCK_H_

#include <vector>
#include <functional>
#include <unordered_map>
#include <string>

#include <any>
#include <flow/Policy.h>

#include <QtCore/QObject>
#include <QBoxLayout>
#include <QIcon>

namespace flow{
    class Outpipe;

    class Block{
    public:
        enum eParameterType {BOOLEAN, INTEGER, DECIMAL, STRING};
        virtual std::string name() const {return "Unnammed";}
        
        __declspec(dllexport) ~Block();

        // BASE METHODS
        virtual bool configure(std::unordered_map<std::string, std::string> _params) { return false; };
        virtual std::vector<std::pair<std::string, eParameterType>> parameters(){ return {}; };

        [[deprecated("This function gives the map with all pipes, please use getPipe method and get just the needed")]]
        __declspec(dllexport) std::unordered_map<std::string, std::shared_ptr<Outpipe>>  getPipes();
        
        __declspec(dllexport) std::shared_ptr<Outpipe> getPipe(std::string _tag);

        __declspec(dllexport) void start();
        __declspec(dllexport) void stop();
        
        // void operator()(std::unordered_map<std::string,std::any> _data, std::unordered_map<std::string,bool> _valid);

        __declspec(dllexport) int nInputs();
        __declspec(dllexport) std::vector<std::string> inputTags();

        __declspec(dllexport) int nOutputs();
        __declspec(dllexport) std::vector<std::string> outputTags();

        __declspec(dllexport) Policy* getPolicy();

        __declspec(dllexport) void connect(std::string _pipeTag, std::string _policyTag, Block& _otherBlock);

        __declspec(dllexport) void disconnect(std::string _pipeTag);

        __declspec(dllexport) virtual QWidget * customWidget() { return nullptr; };
        __declspec(dllexport) virtual QBoxLayout * creationWidget() { return nullptr; };
        
        __declspec(dllexport) virtual bool resizable() const { return false; }

        __declspec(dllexport) virtual std::string description() const {return "Flow block without description";};

        __declspec(dllexport) virtual QIcon icon() const { return QIcon("/usr/share/icons/Humanity/actions/64/help-contents.svg"); };

    protected:
        __declspec(dllexport) bool isRunningLoop() const;
        
        __declspec(dllexport) bool createPipe(std::string _pipeTag, std::string _tagType);
        __declspec(dllexport) bool removePipe(std::string _pipeTag);
        __declspec(dllexport) bool removePipes();

        __declspec(dllexport) bool createPolicy(std::map<std::string, std::string> _inputs);
        __declspec(dllexport) void removePolicy();
        
        __declspec(dllexport) bool registerCallback(Policy::PolicyMask _mask, Policy::PolicyCallback _callback);

    protected:
        virtual void loopCallback() {};

    private:
        Policy *iPolicy_ = nullptr;
        std::unordered_map<std::string, std::shared_ptr<Outpipe>> opipes_;
        std::thread loopThread_;
        bool runLoop_ = false;
    };

}

#endif