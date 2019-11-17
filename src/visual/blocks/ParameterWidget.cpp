
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

#include <flow/visual/blocks/ParameterWidget.h>

//---------------------------------------------------------------------------------------------------------------------
ParameterWidget::ParameterWidget(   const std::string _label, 
                                    const std::string _default, 
                                    QWidget *_parent, 
                                    const char *_name){
    label_ = new QLabel(_label.c_str());

    value_ = new QLineEdit();
    value_->setText(_default.c_str());

    this->addWidget(label_);
    this->addWidget(value_);
}

//---------------------------------------------------------------------------------------------------------------------
ParameterWidget::~ParameterWidget(){
    delete layout_;
    delete label_;
    delete value_;
}

//---------------------------------------------------------------------------------------------------------------------
std::string ParameterWidget::label() const{
    return label_->text().toStdString();
}

//---------------------------------------------------------------------------------------------------------------------
void ParameterWidget::value(const std::string &_value){
    value_->setText(_value.c_str());
}

//---------------------------------------------------------------------------------------------------------------------
std::string ParameterWidget::value() const{
    return value_->text().toStdString();
}


