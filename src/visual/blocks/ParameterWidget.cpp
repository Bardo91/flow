
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

#include <flow/visual/blocks/ParameterWidget.h>
#include <QIntValidator>
#include <QDoubleValidator>
#include <QSpinBox>
#include <QComboBox>

namespace flow{

    std::vector<std::string> split(const std::string& s, char delimiter){
        std::vector<std::string> tokens;
        std::string token;
        std::istringstream tokenStream(s);
        while (std::getline(tokenStream, token, delimiter)) {
            tokens.push_back(token);
        }
        return tokens;
    }

    //---------------------------------------------------------------------------------------------------------------------
    ParameterWidget::ParameterWidget(   const std::string _label, 
                                        flow::Block::eParameterType _type,
                                        const std::string _default, 
                                        QWidget *_parent, 
                                        const char *_name){

        type_ = _type;
        label_ = new QLabel(_label.c_str());

        switch (type_) {
        case flow::Block::eParameterType::STRING:
            value_ = new QLineEdit();
            static_cast<QLineEdit*>(value_)->setText(_default.c_str());
            break;
        case flow::Block::eParameterType::DECIMAL:
            value_ = new QLineEdit();
            static_cast<QLineEdit*>(value_)->setText(_default.c_str());
            static_cast<QLineEdit*>(value_)->setValidator( new QDoubleValidator() );
            break;
        case flow::Block::eParameterType::INTEGER:
        {
            value_ = new QSpinBox();
            std::stringstream ss; ss << _default.c_str();
            int val; ss >> val;
            static_cast<QSpinBox*>(value_)->setValue(val);
            static_cast<QSpinBox*>(value_)->setMaximum(std::numeric_limits<int>::max());
            break;
        }
        case flow::Block::eParameterType::BOOLEAN:
            value_ = new QCheckBox();
            break;
        case flow::Block::eParameterType::OPTIONS:
            value_ = new QComboBox();
            auto values = split(_default, ';');
            for(const auto &value: values){
                static_cast<QComboBox*>(value_)->addItem(value.c_str());
            }
            break;
        }

        this->addWidget(label_);
        this->addWidget(value_);
    }

    //---------------------------------------------------------------------------------------------------------------------
    ParameterWidget::~ParameterWidget(){
        delete label_;
        delete value_;
    }

    //---------------------------------------------------------------------------------------------------------------------
    std::string ParameterWidget::label() const{
        return label_->text().toStdString();
    }


    std::string ParameterWidget::getValueString(){
        return static_cast<QLineEdit*>(value_)->text().toStdString();
    } 

    int ParameterWidget::getValueInt(){
        return static_cast<QSpinBox*>(value_)->value();
    }

    float ParameterWidget::getValueDec(){
        auto str = static_cast<QLineEdit*>(value_)->text().toStdString();
        std::stringstream ss; ss << str;
        float val; ss >> val;
        return val;
    }

    bool ParameterWidget::getValueBool(){
        return static_cast<QCheckBox*>(value_)->isChecked();
    }

    void ParameterWidget::setValueString(std::string _val){
        static_cast<QLineEdit*>(value_)->setText(_val.c_str());
    }

    void ParameterWidget::setValueInt(int _val){
        static_cast<QSpinBox*>(value_)->setValue(_val);
    }

    void ParameterWidget::setValueDec(float _val){
        static_cast<QLineEdit*>(value_)->setText(std::to_string(_val).c_str());
    }

    void ParameterWidget::setValueBool(bool _val){
        static_cast<QCheckBox*>(value_)->setChecked(_val);
    }

}
