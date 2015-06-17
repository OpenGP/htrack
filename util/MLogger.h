#pragma once
#include <iostream>
#include <ostream>
#include <cstdarg>

class MLogger{
    bool _with_space = true;
    bool _with_newline = true;
    bool _with_fatal = false;
    std::string prefix="";
    std::ostream& _out = std::cout;
public:
    enum WriterFlag{space,nospace,fatal,nofatal};
public:
    /// @note default redirects to std::cout
    MLogger(std::ostream& out = std::cout) : _out(out){}
    
    /// @note allows conditional automatic newline
    ~MLogger(){ 
        if(_with_newline) _out << std::endl; 
        if(_with_fatal) exit(255);
    }
    
    /// @note this allows C++ types to be sent to _out
    template <typename T>
    inline MLogger& operator<<(const T& log) {
        _out << log;
        if(_with_space) _out << " ";
        return *this;
    }
       
    /// Allows manipulators
    inline MLogger& operator<<(const WriterFlag& flag){
        switch(flag){
            case space: _with_space=true; break;
            case nospace: _with_space=false; break;
            case fatal: _with_fatal=true; break;
            case nofatal: _with_fatal=false; break;
        }
        return *this;
    }
    
    /// @note this allows mDebug() << ..
    inline MLogger& operator()(){
        return *this;
    }
    
    /// @todo possible alternative: varardic templates and type-safe printf
    inline MLogger& operator()(const char* fmt, ...){
        this->_with_space = false; ///< matches printf
        char buffer[100] = "";
        va_list argptr;
        va_start(argptr,fmt);
        #ifdef _MSC_VER
            vsprintf_s(buffer, fmt, argptr);
        #else
            vsprintf(buffer, fmt, argptr);
        #endif
        va_end(argptr);
        _out << buffer;
        return *this;
    }
    
    static MLogger make_mLogger(){ MLogger out(std::cout); return out; }
    static MLogger make_mDebug(){ MLogger out(std::cout); return out; }
    static MLogger make_mFatal(){ MLogger out(std::cout); out._with_fatal=true; out<<"!!!FATAL:"; return out; }
    static MLogger make_mWarning(){ MLogger out(std::cout); out<<"!!!WARNING:"; return out; }
};

/// Conditionally enables Qt support
#ifdef QT_CORE_LIB
    #include <QString>
    inline MLogger &operator<<(MLogger& d, const QString& t) { d<<t.toStdString(); return d; }
    /// @todo add it elegantly. useful to see content of packed Hex
    // qDebug() << QString("%1").arg(value, 8, 16, QLatin1Char('0'));
#endif

#define mLogger  MLogger::make_mLogger()
#define mFatal   MLogger::make_mFatal()
#define mDebug   MLogger::make_mDebug()
#define mWarning MLogger::make_mWarning()

#define TRACE() mDebug() << "[TRACE] line" << __LINE__ << "@ " << __FILE__;
#define CHECK(test) if(!(test)){ mDebug() << "[CHECK]" << #test << "failed at line" << __LINE__ << "@" << __FILE__; exit(255); }
#define CHECK_NOTNULL(ptr) if((ptr)==NULL){ mDebug() << "!!!FATAL " << #ptr << "is null"; exit(255); }
