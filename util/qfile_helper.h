#include <QCoreApplication>
#include <QDir>
#include <QString>
#include <string>
#include "mylogger.h"

/// Computes the full file path of a file with a relative location given w.r.t. the EXE file
inline std::string local_file_path(const std::string& local_path, bool exit_if_notfound=true){
    QString exe_path = QCoreApplication::applicationDirPath();
    QString qpath = QDir(exe_path).filePath(local_path.c_str());
    bool exists = QDir(qpath).exists() || QFile(qpath).exists();
    if(!exists){
        if(exit_if_notfound){
            LOG(INFO) << "!!!FATAL: failed to find resource: " << qpath;
            exit(EXIT_FAILURE);
        } else {
            return "";
        }
    }
    return qpath.toStdString();
}
