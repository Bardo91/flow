//---------------------------------------------------------------------------------------------------------------------
//  FLOW
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2019 - Pablo Ramon Soria (a.k.a. Bardo91) 
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

#include <flow/Policy.h>
#include <flow/OutPipe.h>

#include <gtest/gtest.h>
#include <condition_variable> 

using namespace mico;

TEST(stream_creation, stream_creation)  {
    Policy pol1({"i1"});
    Policy pol2({"i1","i2"});
    Policy pol3({"i1","i2","i3"});
    Policy pol4({""});
    EXPECT_THROW(Policy pol5({}), std::invalid_argument);
}

TEST(pipe_creation, pipe_creation)  {
    
    OutPipe op1("int");
    OutPipe op2("float");
    Policy pol({"int"});

    ASSERT_TRUE(op1.registerPolicy(&pol));
    ASSERT_FALSE(op2.registerPolicy(&pol));

}

TEST(registration, registration)  {
    // Invalid creation
    EXPECT_THROW(OutPipe op(""), std::invalid_argument);

    // Valid creation
    OutPipe op("o1");
    ASSERT_STREQ("o1", op.tag().c_str());
    ASSERT_EQ(0, op.registrations());

    Policy pol({"o1"});
    ASSERT_TRUE(op.registerPolicy(&pol));
    ASSERT_FALSE(op.registerPolicy(&pol));
    ASSERT_EQ(1, op.registrations());

}

TEST(transmission_int_1, transmission_int)  {
    OutPipe op("int");

    Policy pol({"int"});

    int res = 0;
    pol.registerCallback({"int"}, [&](std::unordered_map<std::string,std::any> _data){
        res = std::any_cast<int>(_data["int"]);
    });

    op.registerPolicy(&pol);

    // Good type flush
    op.flush(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ASSERT_EQ(res, 1);

    // Bad type flush
    EXPECT_THROW(
        op.flush(5.312f);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    , std::bad_any_cast);
}


TEST(disconnect, disconnect)  {
    OutPipe op("int");
    Policy pol({"int"});

    int res = 0;
    bool goodCallback = pol.registerCallback({"int"}, [&](std::unordered_map<std::string,std::any> _data){
        res = std::any_cast<int>(_data["int"]);
    });
    ASSERT_TRUE(goodCallback);

    bool badCallback = pol.registerCallback({"float"}, [&](std::unordered_map<std::string,std::any> _data){ });
    ASSERT_FALSE(badCallback);
    
    op.registerPolicy(&pol);
    
    op.flush(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ASSERT_EQ(res, 1);

    pol.disconnect("int");

    op.flush(2);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ASSERT_EQ(res, 1);
    ASSERT_EQ(0, op.registrations());


}

TEST(transmission_int_2, transmission_int)  {
    OutPipe op("int");

    Policy pol1({"int"});
    Policy pol2({"int"});

    int counterCall1 = 0;
    std::mutex guardCall1;
    pol1.registerCallback({"int"}, [&](std::unordered_map<std::string,std::any> _data){
        guardCall1.lock();
        counterCall1++;
        guardCall1.unlock();    
    });
    
    int counterCall2 = 0;
    std::mutex guardCall2;
    pol2.registerCallback({"int"}, [&](std::unordered_map<std::string,std::any> _data){
        guardCall2.lock();
        counterCall2++;
        guardCall2.unlock();
    });

    op.registerPolicy(&pol1);
    op.registerPolicy(&pol2);

    // Good type flush
    op.flush(1);
    op.flush(1);
    op.flush(1);
    op.flush(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    ASSERT_EQ(4, counterCall1);
    ASSERT_EQ(4, counterCall2);
}


TEST(sync_policy, sync_policy)  {
    OutPipe op1("int");
    OutPipe op2("float");

    Policy pol({"int", "float"});

    int counterCallInt = 0;
    int counterCallFloat = 0;
    int counterCallSync = 0;
    std::mutex guardCall1;
    pol.registerCallback({"int"}, [&](std::unordered_map<std::string,std::any> _data){
        counterCallInt++;    
    });
    pol.registerCallback({"float"}, [&](std::unordered_map<std::string,std::any> _data){
        counterCallFloat++;    
    });
    pol.registerCallback({"int", "float"}, [&](std::unordered_map<std::string,std::any> _data){
        counterCallSync++;    
    });
    
    op1.registerPolicy(&pol);
    op2.registerPolicy(&pol);

    // Good type flush
    op1.flush(1);
    op2.flush(1.123f);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    ASSERT_EQ(1, counterCallInt);
    ASSERT_EQ(1, counterCallFloat);
    ASSERT_EQ(1, counterCallSync);
}



TEST(deep_chain, deep_chain)  {
    OutPipe op("int");
    Policy pol({"int"});
    op.registerPolicy(&pol);

    OutPipe op2("int");
    Policy pol2({"int"});
    op2.registerPolicy(&pol2);

    OutPipe op3("int");
    Policy pol3({"int"});
    op3.registerPolicy(&pol3);


    pol.registerCallback({"int"}, [&](std::unordered_map<std::string,std::any> _data){
        int res = 2 * std::any_cast<int>(_data["int"]);
        ASSERT_EQ(res, 4);
        op2.flush(res);
    });

    pol2.registerCallback({"int"}, [&](std::unordered_map<std::string,std::any> _data){
        int res = 2 * std::any_cast<int>(_data["int"]);
        ASSERT_EQ(res, 8);
        op3.flush(res);
    });

    bool called = false;
    pol3.registerCallback({"int"}, [&](std::unordered_map<std::string,std::any> _data){
        int res = 2 * std::any_cast<int>(_data["int"]);
        ASSERT_EQ(res, 16);
        called = true;  
    });
    
    op.flush(2);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ASSERT_TRUE(called);
}


TEST(deep_chain_split, deep_chain_split)  {
    //
    // Structure of test
    //                             |--> o2 --> p2 --> o4 --> p4
    //  o0 --> p0 --> o1 --> p1 -->|
    //                             |--> o3 --> p3 --> o5 --> p5
    //
    //  2------4--------------8----------------16------------32------Check here

    std::vector<OutPipe*> pipes;
    std::vector<Policy*> policies;

    for(unsigned i = 0; i < 7; i++){
        pipes.push_back(new OutPipe("int"));
        policies.push_back(new Policy({"int"}));
        pipes.back()->registerPolicy(policies.back());
    }


    policies[0]->registerCallback({"int"}, [&](std::unordered_map<std::string,std::any> _data){
        int res = 2 * std::any_cast<int>(_data["int"]);
        pipes[1]->flush(res);
    });

    policies[1]->registerCallback({"int"}, [&](std::unordered_map<std::string,std::any> _data){
        int res = 2 * std::any_cast<int>(_data["int"]);
        pipes[2]->flush(res);
        pipes[3]->flush(res);
    });

    // Branch 1
    policies[2]->registerCallback({"int"}, [&](std::unordered_map<std::string,std::any> _data){
        int res = 2 * std::any_cast<int>(_data["int"]);
        pipes[4]->flush(res);
    });

    int nCounterB1 = 0;
    std::mutex lockerB1;
    policies[4]->registerCallback({"int"}, [&](std::unordered_map<std::string,std::any> _data){
        lockerB1.lock();
        int res = 2 * std::any_cast<int>(_data["int"]);
        ASSERT_EQ(res, 32);
        nCounterB1++;
        lockerB1.unlock();
    });

    // Branch 2
    policies[3]->registerCallback({"int"}, [&](std::unordered_map<std::string,std::any> _data){
        int res = 2 * std::any_cast<int>(_data["int"]);
        pipes[5]->flush(res);
    });

    int nCounterB2 = 0;
    std::mutex lockerB2;
    policies[5]->registerCallback({"int"}, [&](std::unordered_map<std::string,std::any> _data){
        lockerB2.lock();
        int res = 2 * std::any_cast<int>(_data["int"]);
        ASSERT_EQ(res, 32);
        nCounterB2++;
        lockerB2.unlock();
    });

    
    pipes.front()->flush(2);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    ASSERT_EQ(nCounterB1, 1);
    ASSERT_EQ(nCounterB2, 1);
}

TEST(loop_chain_split, loop_chain_split)  {
    //
    // Structure of test
    //                           
    //         v--------------------------| 
    //  o0 --> p0 --> o1 --> p1 -|--> o3 -|-> p3
    //                           |
    //                           |--> o4 --> p4

    // Create pipes and connections
    OutPipe o0("int");
    Policy p0({"int", "string"});
    o0.registerPolicy(&p0);

    OutPipe o1("int");
    Policy p1({"int"});
    o1.registerPolicy(&p1);

    OutPipe o3("string");
    Policy p3({"string"});
    o3.registerPolicy(&p0);
    o3.registerPolicy(&p3);

    OutPipe o4("int");
    Policy p4({"int"});
    o4.registerPolicy(&p4);

    // Set callbacks
    bool calledOnce0a = true;
    std::mutex guard0a;
    p0.registerCallback({"int"}, [&](std::unordered_map<std::string,std::any> _data){
        guard0a.lock();
        o1.flush(1);
        ASSERT_TRUE(calledOnce0a);
        calledOnce0a = false;
        guard0a.unlock();
    });

    bool calledOnce0b = true;
    std::mutex guard0b;
    p0.registerCallback({"string"}, [&](std::unordered_map<std::string,std::any> _data){
        guard0b.lock();
        ASSERT_TRUE(calledOnce0b);
        calledOnce0b = false;
        guard0b.unlock();
    });

    p1.registerCallback({"int"}, [&](std::unordered_map<std::string,std::any> _data){
        o3.flush("pepe");
        o4.flush(1);
    });

    bool calledOnce4 = true;
    std::mutex guard4;
    p4.registerCallback({"int"}, [&](std::unordered_map<std::string,std::any> _data){
        guard4.lock();
        ASSERT_TRUE(calledOnce4);
        calledOnce4 = false;
        guard4.unlock();
    });

    o0.flush(1);
    while(calledOnce4){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}



TEST(concurrency_attack_test, concurrency_attack_test)  {
    OutPipe op("int");

    Policy pol({"int"});

    int counterCall1 = 0;
    bool idle = true;
    pol.registerCallback({"int"}, [&](std::unordered_map<std::string,std::any> _data){
        if(idle){
            idle=false;
            counterCall1++;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));    // To ensure that no one else is comming in
            // std::cout << "jeje, I am the one which got into " << std::this_thread::get_id() << std::endl;
            idle=true;
        }else{
            // std::cout << "Shit I failed" << std::this_thread::get_id() << std::endl;
        }
    });

    op.registerPolicy(&pol);    

    std::mutex mtx;
    std::condition_variable cv;
    bool ready = false;

    auto print_id = [&](int id) {
        std::unique_lock<std::mutex> lck(mtx);
        while (!ready) cv.wait(lck);
        op.flush(1);
    };

    std::thread vThreads[100];
    for (int i=0; i<100; ++i)
        vThreads[i] = std::thread(print_id,i);

    {
        std::unique_lock<std::mutex> lck(mtx);
        ready = true;
        cv.notify_all();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    for (auto& th : vThreads) th.join();

    ASSERT_EQ(1, counterCall1);
}


 
int main(int _argc, char **_argv)  {
    testing::InitGoogleTest(&_argc, _argv);
    return RUN_ALL_TESTS();
}