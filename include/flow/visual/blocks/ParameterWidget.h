
//---------------------------------------------------------------------------------------------------------------------
//  flow
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

#ifndef FLOW_VISUAL_BLOCKS_PARAMETERWIDGET_H_
#define FLOW_VISUAL_BLOCKS_PARAMETERWIDGET_H_

#include <flow/Export.h>

#include <QWidget>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QCheckBox>
#include <QLabel>

#include <flow/Block.h>
#include <sstream>
namespace flow{

    class ParameterWidget: public QHBoxLayout{
    public:
        ParameterWidget(const std::string _label,
                        Block::eParameterType _type,
                        const std::string _default, 
                        QWidget *_parent = nullptr, 
                        const char *_name = nullptr);
        ~ParameterWidget();
        
        std::string label() const;

        std::string getValueString();
        int getValueInt();
        float getValueDec();
        bool getValueBool();

        void setValueString(std::string _val);
        void setValueInt(int _val);
        void setValueDec(float _val);
        void setValueBool(bool _val);

        Block::eParameterType type() { return type_; };

    private:
        QLabel   * label_;
        QWidget   * value_;
        flow::Block::eParameterType type_;

    };


}

#endif