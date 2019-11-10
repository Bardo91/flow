
//---------------------------------------------------------------------------------------------------------------------
//  flow
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


#ifndef FLOW_VISUAL_CODEGENERATION_CODEGENERATOR_H_
#define FLOW_VISUAL_CODEGENERATION_CODEGENERATOR_H_

#include <QJsonObject>

#include <fstream>

#include <unordered_map>

namespace flow{
    class CodeGenerator{
    public:
        static void parseScene(std::string _cppFile, QJsonObject const &_scene);
        static void generateCmake(std::string _cmakeFile, std::string _cppName);
        static void compile(std::string _cppFolder);
    private:
        static void writeInit(std::ofstream &_file);
        static void writeEnd(std::ofstream &_file);

        static std::string demangleClassType(const char* mangled);
    
        static std::unordered_map<std::string, std::string> dictClassInit;
    };
}

#endif