
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

#ifndef FLOW_VISUAL_BLOCKS_FLOWBLOCK_H_
#define FLOW_VISUAL_BLOCKS_FLOWBLOCK_H_

#include <flow/flow.h>

#include <nodes/NodeDataModel>
#include <nodes/Connection>

#include <QtCore/QObject>
#include <QtWidgets/QLineEdit>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QPushButton>
#include <QCheckBox>

#include <iostream>

using QtNodes::NodeData;
using QtNodes::NodeDataModel;
using QtNodes::NodeDataType;
using QtNodes::PortIndex;
using QtNodes::PortType;
using QtNodes::Connection;

namespace flow{

    /// Forward declaration
    class OutPipe;

    /// Interface to allow static cast of MicoBlocks.
    class ConfigurableBlock{
    public:
        virtual void configure() = 0;

        void julluar(){
            std::cout << "I am a configurable block" << std::endl;
        }
    };

    /// 
    template<typename Block_, bool HasAutoLoop_ = false>
    class FlowVisualBlock : public NodeDataModel, public ConfigurableBlock {
    public:
        FlowVisualBlock();

        virtual ~FlowVisualBlock();
        
        QJsonObject save() const override;
        void restore(QJsonObject const &p) override;

        std::unordered_map<std::string, std::string> extractParamsGui();

        void configure() override;

        flow::Block * internalBlock() const; 
    public:
        QString caption() const override { return Block_::name().c_str(); }

        bool captionVisible() const override { return true; }

        static QString Name() { return Block_::name().c_str(); }

        QString name() const override { return Block_::name().c_str(); }
    
        virtual void inputConnectionDeleted(Connection const&) override;

    public:
        unsigned int nPorts(PortType portType) const override;

        NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

        std::shared_ptr<NodeData> outData(PortIndex port) override;

        void setInData(std::shared_ptr<NodeData> data, PortIndex port) override;

        QWidget * embeddedWidget() override { return configBox_; }

    private:
        Block_ *flowBlock_;
        //std::unordered_map<std::string, OutPipe*> connectedPipes_;
        std::vector<QLineEdit*> configLabels_;
        QVBoxLayout *configsLayout_  = nullptr;
        QGroupBox *configBox_ = nullptr;
        QPushButton *configButton_  = nullptr;
        QCheckBox *streamActionButton_ = nullptr;

    };
}


#include <flow/visual/blocks/MicoFlowBlock.inl>

#endif