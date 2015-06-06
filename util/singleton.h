#pragma once
#ifndef SINGLETON
#define SINGLETON(ClassName)\
private: \
    ClassName() {} \
    ClassName(ClassName const&); \
    void operator=(ClassName const&); \
public: \
    static ClassName& instance(){ \
        static ClassName _instance; \
        return _instance; \
    }  
#endif 
