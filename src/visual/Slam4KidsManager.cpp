
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


#ifdef HAS_QTNODEEDITOR

#include <flow/visual/Slam4KidsManager.h>

#include <nodes/NodeData>
#include <nodes/Node>
#include <nodes/FlowScene>
#include <nodes/FlowView>

#include <QtWidgets/QApplication>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QFileDialog>
#include <QtCore/QByteArray>
#include <QtCore/QBuffer>
#include <QtCore/QDataStream>
#include <QtCore/QFile>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>

#ifdef FLOW_USE_ROS
	#include <ros/ros.h>
#endif
#ifdef foreach  // To be able to use Qt and RealSense Device
  #undef foreach
#endif

#include <flow/flow.h>
#include <flow/visual/blocks/MicoFlowBlock.h>
#include <flow/visual/code_generation/CodeGenerator.h>


using QtNodes::FlowView;
using QtNodes::FlowScene;

namespace flow{
    int Slam4KidsManager::init(int _argc, char** _argv){
        kids_app = new QApplication(_argc, _argv);

        #ifdef FLOW_USE_ROS
        	ros::init(_argc, _argv, "SLAM4KIDS");
        	ros::AsyncSpinner spinner(4);
        	spinner.start();
		#endif

        QWidget mainWidget;
        auto menuBar    = new QMenuBar();
        auto saveAction = menuBar->addAction("Save");
        auto loadAction = menuBar->addAction("Load");
        auto configureAll = menuBar->addAction("Configure All");
        auto generateCode = menuBar->addAction("Generate Code");

        QVBoxLayout *l = new QVBoxLayout(&mainWidget);
        l->addWidget(menuBar);
        auto scene = new FlowScene(     registerDataModels(), 
                                        &mainWidget
                                        );

        l->addWidget(new FlowView(scene));
        l->setContentsMargins(0, 0, 0, 0);
        l->setSpacing(0);

        QObject::connect(saveAction, &QAction::triggered, scene, &FlowScene::save);

        QObject::connect(loadAction, &QAction::triggered, scene, &FlowScene::load);


        QObject::connect(configureAll, &QAction::triggered, [&](){
            auto nodes = scene->allNodes();

            for(auto node:nodes){
                NodeDataModel* dataModel = node->nodeDataModel();
                // This conversion is not safe but, all nodes in slam4kids are ConfigurableBlocks
                auto d_ptr = dynamic_cast<ConfigurableBlock*>(dataModel);
                if(d_ptr != nullptr)
                    d_ptr->configure();
            }

        });

        QObject::connect(generateCode, &QAction::triggered, [](){
            QString fileName = QFileDialog::getOpenFileName(nullptr,
                                                "Select scene to save",
                                                QDir::homePath(),
                                                "Flow Scene Files (*.flow)");

            if (!QFileInfo::exists(fileName))
            return;

            QFile file(fileName);

            if (!file.open(QIODevice::ReadOnly))
            return;

            std::string cppFile = fileName.toStdString().substr(0, fileName.size()-4) + "cpp";
            auto lastBar = cppFile.find_last_of('/');
            std::string cppFolder = cppFile.substr(0, lastBar);
            std::string cmakeFilePath = cppFolder + "/CMakeLists.txt";


            QByteArray wholeFile = file.readAll();
            QJsonObject const jsonDocument = QJsonDocument::fromJson(wholeFile).object();
            CodeGenerator::parseScene(cppFile,jsonDocument);
            CodeGenerator::generateCmake(cmakeFilePath, cppFile);
            CodeGenerator::compile(cppFolder);


        });

        mainWidget.setWindowTitle("Node-based flow editor");
        mainWidget.resize(800, 600);
        mainWidget.showNormal();

        return kids_app->exec();
    }


    void  Slam4KidsManager::quit(){
        kids_app->quit();
    }


    void Slam4KidsManager::setNodeRegisterFn(std::function<void(std::shared_ptr<QtNodes::DataModelRegistry> &_registry)> _fn){
        registerFn_ = _fn;
    }

    std::shared_ptr<QtNodes::DataModelRegistry> Slam4KidsManager::registerDataModels(){
        auto registry = std::make_shared<QtNodes::DataModelRegistry>();

        registerFn_(registry);

        return registry;
    }

}


#endif