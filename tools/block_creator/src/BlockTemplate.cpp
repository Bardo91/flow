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

#include <block_creator/BlockTemplate.h>


namespace flow{
    namespace creator{
        void BlockTemplate::generate(const std::string &_path){

        }

        void BlockTemplate::name(const std::string &_name){
            name_ = _name;
        }


        std::string BlockTemplate::name() const{
            return name_;
        }


        void BlockTemplate::inputs(const std::vector<BlockTemplate::InputOutputInfo> &_inputs){
            inputs_ = _inputs;
        }


        std::vector<BlockTemplate::InputOutputInfo> BlockTemplate::inputs() const{
            return inputs_;
        }


        void BlockTemplate::outputs(const std::vector<BlockTemplate::InputOutputInfo> &_outputs){
            outputs_ = _outputs;
        }


        std::vector<BlockTemplate::InputOutputInfo> BlockTemplate::outputs() const{
            return outputs_;
        }


        void BlockTemplate::callbacks(const std::vector<std::string> &_callbacks){
            callbacks_ = _callbacks;
        }


        std::vector<std::string> BlockTemplate::callbacks() const{
            return callbacks_;
        }

    }
}