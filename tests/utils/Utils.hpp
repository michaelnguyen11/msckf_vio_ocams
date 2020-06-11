#ifndef VIO_TEST_UTILS_HPP_
#define VIO_TEST_UTILS_HPP_

#include <sys/stat.h>
#include <sys/types.h>

#include <cstring>
#include <string>
#include <vector>

// modified after Tam Do
namespace fs {
static bool fileExist(const std::string &path) {
    bool isExist = false;
    struct stat info;
    if (stat(path.c_str(), &info) == 0) {
        if (info.st_mode & S_IFREG) {
            isExist = true;
        }
    }

    return isExist;
}

static bool dirExist(const std::string &path) {
    bool isExist = false;
    struct stat info;
    if (stat(path.c_str(), &info) == 0) {
        if (info.st_mode & S_IFDIR) {
            isExist = true;
        }
    }

    return isExist;
}

static bool createDirectory(const std::string &path) {
    return (mkdir(path.c_str(), 0777) == 0);
}

}  // namespace fs

namespace str {

// Modified from: https://github.com/tobbez/string-splitting/blob/master/splitc3.cpp
// With: https://stackoverflow.com/questions/7352099/stdstring-to-char
// how to use:
// std::vector<std::string> l_str0;
// str::split(l_str0, f0_line, ",");
static void split(std::vector<std::string> &tokens, std::string str,
        const char *delimiters) {
    char *cstr = new char[str.length() + 1];
    strcpy(cstr, str.c_str());
    char *saveptr;
    char *token;

    for(token = strtok_r(cstr, delimiters, &saveptr);
        token != NULL;
        token = strtok_r(NULL, delimiters, &saveptr)) {
        tokens.push_back(std::string(token));
    }

    delete [] cstr;
}

}  // namespace str

#endif