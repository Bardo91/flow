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

#include <flow/Export.h>
#include <flow/Persistency.h>

#include <vector>
#include <functional>
#include <unordered_map>
#include <string>

#include <any>
#include <flow/Policy.h>
#include <flow/Outpipe.h>

#include <tuple>
#include <any>

#include <QtCore/QObject>
#include <QBoxLayout>
#include <QIcon>
#include <cassert>
#include<optional>

namespace flow{

    struct ConfigParameterDef {
        enum eParameterType { BOOLEAN, INTEGER, DECIMAL, STRING, OPTIONS };
        std::string name_;
        eParameterType type_;
        std::any value_;
        bool                     asBool()       const { assert(type_ == eParameterType::BOOLEAN); return std::any_cast<bool>(value_); };
        int                      asInteger()    const { assert(type_ == eParameterType::INTEGER); return std::any_cast<int>(value_); };
        float                    asDecimal()    const { assert(type_ == eParameterType::DECIMAL); return std::any_cast<float>(value_); };
        std::string              asString()     const { assert(type_ == eParameterType::STRING);  return std::any_cast<std::string>(value_); };
        std::vector<std::string> asOptions()    const { assert(type_ == eParameterType::OPTIONS); return std::any_cast<std::vector<std::string>>(value_); };
    };

    class Block{
    public:
        virtual std::string name() const {return "Unnammed";}
        
        ~Block();

        // BASE METHODS
        virtual bool configure(std::vector<flow::ConfigParameterDef> _params) { return false; };
        virtual std::vector<flow::ConfigParameterDef> parameters(){ return {}; };

        [[deprecated("This function gives the map with all pipes, please use getPipe method and get just the needed")]]
        std::unordered_map<std::string, std::shared_ptr<Outpipe>>  getPipes();
        
        std::shared_ptr<Outpipe> getPipe(std::string _tag);

        void start();
        void stop();
        
        // void operator()(std::unordered_map<std::string,std::any> _data, std::unordered_map<std::string,bool> _valid);

        int nInputs();
        std::vector<std::string> inputTags();

        int nOutputs();
        std::vector<std::string> outputTags();

        Policy* getPolicy();

        void connect(std::string _pipeTag, std::string _policyTag, Block& _otherBlock);

        void disconnect(std::string _pipeTag);

        virtual QWidget * customWidget() { return nullptr; };
        virtual QBoxLayout * creationWidget() { return nullptr; };
        
        virtual bool resizable() const { return false; }

        virtual std::string description() const {return "Flow block without description";};

        virtual QIcon icon() const { return QIcon((Persistency::resourceDir()+"question.svg").c_str()); };

        std::optional<ConfigParameterDef> getParamByName(const std::vector<flow::ConfigParameterDef> &_params, const std::string &_pname);

    protected:
        bool isRunningLoop() const;
        
        template<typename T_>
        bool createPipe(std::string _pipeTag);
        bool removePipe(std::string _pipeTag);
        bool removePipes();

        bool createPolicy(std::vector<PolicyInput*> _inputs);
        void removePolicy();
        
        bool registerCallback(Policy::PolicyMask _mask, Policy::PolicyCallback _callback);

    protected:
        virtual void loopCallback() {};

    private:
        Policy *iPolicy_ = nullptr;
        std::unordered_map<std::string, std::shared_ptr<Outpipe>> opipes_;
        std::thread loopThread_;
        bool runLoop_ = false;
    };


    template<typename T_>
    bool Block::createPipe(std::string _pipeTag){
        if(opipes_[_pipeTag] == nullptr){
            opipes_[_pipeTag] = std::shared_ptr<Outpipe>(new flow::Outpipe(_pipeTag, typeid(T_).name()));
            return true;
        }else{
            throw std::invalid_argument("Pipe with tag " + _pipeTag + " already defined.");
            return false;
        }
    }



}

#endif