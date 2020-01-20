
//---------------------------------------------------------------------------------------------------------------------
//  flow
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


#include <flow/visual/data_types/StreamerPipeInfo.h>
#include <QJsonArray>
#include <QIcon>


namespace flow{

    template<typename Block_, bool HasAutoLoop_>
    inline FlowVisualBlock<Block_,HasAutoLoop_>::FlowVisualBlock() {
        flowBlock_ = new Block_();

        // Configure group box
        configsLayout_ = new QVBoxLayout();
        configBox_ = new QGroupBox("");
        configBox_->setLayout(configsLayout_);
        // Configure frequencies
        freqsLayout_ = new QHBoxLayout();
        configsLayout_->addLayout(freqsLayout_);
        if(flowBlock_->getPolicy()){
            freqsLayout_->addWidget(new QLabel("Freqs: "));
            for(unsigned i = 0; i < flowBlock_->getPolicy()->masksFrequencies().size(); i++){
                freqLabels_.push_back(new QLabel("0"));
                freqsLayout_->addWidget(freqLabels_.back());
            }
            if(freqLabels_.size() > 0){
                freqLabelUpdater_ = std::thread([&](){
                    while(flowBlock_){
                        auto freqs = flowBlock_->getPolicy()->masksFrequencies();
                        for(unsigned i = 0; i < freqs.size(); i++){
                            freqLabels_[i]->setText(QString::number(freqs[i], 'f', 2));
                        }
                        std::this_thread::sleep_for(std::chrono::milliseconds(300));
                    }
                });
            }
        }
        
        // custom visualizer
        if(flowBlock_->customWidget() != nullptr){
            configsLayout_->addWidget(flowBlock_->customWidget());
        }

        // configure parameters
        if(flowBlock_->parameters().size() > 0){
            for(auto &param: flowBlock_->parameters()){
                configParams_.push_back(new ParameterWidget(param.c_str(), ""));
                configsLayout_->addLayout(configParams_.back());
            }
            configButton_ = new QToolButton();
            configButton_->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
            configButton_->setLayoutDirection(Qt::RightToLeft);
            configButton_->setIcon(QIcon("/usr/share/icons/Humanity/actions/128/help-contents.svg"));
            configButton_->setText("Configure");
            configsLayout_->addWidget(configButton_);
            connect(configButton_, &QPushButton::clicked, this, [this]() {
                this->configure();
            });
            //Configure configure states
            configStateIcon_ = new QIcon();
        }

        // Autoloop button
        if(HasAutoLoop_){
            streamActionButton_ = new QCheckBox("Run");
            configsLayout_->addWidget(streamActionButton_);
            connect(    streamActionButton_, &QCheckBox::toggled,
                        [=](bool checked) { 
                                if(checked){
                                    flowBlock_->start();
                                }else{
                                    flowBlock_->stop();
                                }
                            });
        }


    }

    template<typename Block_, bool HasAutoLoop_>
    inline FlowVisualBlock<Block_,HasAutoLoop_>::~FlowVisualBlock(){
        delete flowBlock_;
        flowBlock_ = nullptr;
        if(freqLabelUpdater_.joinable())
            freqLabelUpdater_.join();
    }


    template<typename Block_, bool HasAutoLoop_>
    inline QJsonObject FlowVisualBlock<Block_,HasAutoLoop_>::save() const{
        QJsonObject modelJson = NodeDataModel::save();

        unsigned counter = 0;
        QJsonObject jsonParams;
        for(auto &param: flowBlock_->parameters()){
            jsonParams[param.c_str()] =  configParams_[counter]->value().c_str();
            counter++;
        }
        modelJson["params"] = jsonParams;

        modelJson["class"] = typeid(flowBlock_).name();

        // Save input tags
        std::vector<std::string> inTags = flowBlock_->inputTags();
        QJsonArray jsonInTags;
        for(auto &tag:inTags){
            jsonInTags.append(tag.c_str());
        }
        modelJson["in_tags"] = jsonInTags;

        // Save output tags
        std::vector<std::string> outTags = flowBlock_->outputTags();
        QJsonArray jsonOutTags;
        for(auto &tag:outTags){
            jsonOutTags.append(tag.c_str());
        }
        modelJson["out_tags"] = jsonOutTags;

        modelJson["autoloop"] = HasAutoLoop_;    

        return modelJson;
    }
 
    template<typename Block_, bool HasAutoLoop_>
    inline std::unordered_map<std::string, std::string> FlowVisualBlock<Block_,HasAutoLoop_>::extractParamsGui(){
        std::unordered_map<std::string, std::string> params;
        int counter = 0; 
        for(auto &param: flowBlock_->parameters()){
            params[param] =  configParams_[counter]->value();
            counter++;
        }

        return params;
    }

    template<typename Block_, bool HasAutoLoop_>
    inline void FlowVisualBlock<Block_,HasAutoLoop_>::restore(QJsonObject const &_json) {
        
        unsigned counter = 0;
        for(auto &param: flowBlock_->parameters()){
            QJsonValue v = _json["params"].toObject()[param.c_str()];
            if (!v.isUndefined()) {
                QString strNum = v.toString();
                configParams_[counter]->value(strNum.toStdString());
            }
            counter++;
        }
    }


    template<typename Block_, bool HasAutoLoop_>
    inline void FlowVisualBlock<Block_,HasAutoLoop_>::configure(){
        if(flowBlock_->configure(this->extractParamsGui())){
            if(configButton_) 
                configButton_->setIcon(QIcon("/usr/share/icons/Humanity/actions/48/dialog-apply.svg"));
        }else{
            if(configButton_)
                configButton_->setIcon(QIcon("/usr/share/icons/Humanity/actions/64/process-stop.svg"));
        }
    }

    template<typename Block_, bool HasAutoLoop_>
    inline flow::Block * FlowVisualBlock<Block_,HasAutoLoop_>::internalBlock() const{
        return flowBlock_;
    }

    template<typename Block_, bool HasAutoLoop_>
    inline void FlowVisualBlock<Block_,HasAutoLoop_>::inputConnectionDeleted(Connection const&_conn) {
        // Unregister element in policy
        auto tag = flowBlock_->getPolicy()->inputTags()[_conn.getPortIndex(PortType::In)];
        flowBlock_->disconnect(tag);
        
    }

    template<typename Block_, bool HasAutoLoop_>
    inline unsigned int FlowVisualBlock<Block_,HasAutoLoop_>::nPorts(PortType portType) const {
        unsigned int result = 0;

        switch (portType) {
        case PortType::In:
            result = flowBlock_->nInputs();
            break;
        case PortType::Out:
            result = flowBlock_->nOutputs();
        default:
            break;
        }

        return result;
    }

    template<typename Block_, bool HasAutoLoop_>
    inline NodeDataType FlowVisualBlock<Block_,HasAutoLoop_>::dataType(PortType portType, PortIndex index) const {
        std::vector<std::string> tags;
        if(portType == PortType::In){
            tags = flowBlock_->inputTags();
        }else{
            tags = flowBlock_->outputTags();
        }

        assert(unsigned(index) < tags.size());
        
        auto iter = tags.begin() + index;
        std::string tag = *iter;

        // In NodeEditor, NodeDataType is just a pair of string indicating type and name of type. So faking it.
        //return StreamerPipeInfo(nullptr, tag).type();
        
        if(portType == PortType::In){
            auto type = flowBlock_->getPolicy()->type(tag);
            // std::cout << Block_::name() << ". " << index << ": " << type << std::endl;
            return NodeDataType{type.c_str(),tag.c_str()};
        }else{
            auto type = flowBlock_->getPipe(tag)->type();
            // std::cout << Block_::name() << ". " << index << ": " << type << std::endl;
            return NodeDataType{type.c_str(),tag.c_str()};
        }
    }

    template<typename Block_, bool HasAutoLoop_>
    inline std::shared_ptr<NodeData> FlowVisualBlock<Block_,HasAutoLoop_>::outData(PortIndex index) {
        auto tag = flowBlock_->outputTags()[index];
        std::shared_ptr<StreamerPipeInfo> ptr(new StreamerPipeInfo(flowBlock_, tag));  // 666 TODO
        return ptr;
    }


    template<typename Block_, bool HasAutoLoop_>
    inline void FlowVisualBlock<Block_,HasAutoLoop_>::setInData(std::shared_ptr<NodeData> data, PortIndex port) {
        // 666 Connections do not transfer data but streamers information to connect to internal block.
        if(data){
            auto pipeInfo = std::dynamic_pointer_cast<StreamerPipeInfo>(data)->info();
            if(pipeInfo.otherBlock_ != nullptr){
                pipeInfo.otherBlock_->connect(pipeInfo.pipeName_, flowBlock_->inputTags()[port], *flowBlock_);
            }
        }
    }

}