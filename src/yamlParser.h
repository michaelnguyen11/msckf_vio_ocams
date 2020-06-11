#ifndef YAML_PARSER_H_
#define YAML_PARSER_H_

#include <fstream>
#include <memory>
#include <string>
#include <glog/logging.h>

#include <opencv2/core/core.hpp>

namespace msckf_vio
{

class YamlParser
{
public:
    typedef std::shared_ptr<YamlParser> Ptr;

    YamlParser(const std::string &filePath) : _fs(), _filePath(filePath)
    {
        this->openFile(filePath, &_fs);
    }
    ~YamlParser()
    {
        this->closeFile(&_fs);
    }

    template <class T>
    void getYamlParam(const std::string &id, T *output) const
    {
        CHECK(!id.empty());
        const cv::FileNode &fileHandle = _fs[id];
        CHECK_NE(fileHandle.type(), cv::FileNode::NONE) << "Missing parameter: " << id.c_str() << " in file: " << _filePath.c_str();
        fileHandle >> *CHECK_NOTNULL(output);
    }

    template <class T>
    void getNestedYamlParam(const std::string &id, const std::string &id2, T *output) const
    {
        CHECK(!id.empty());
        CHECK(!id2.empty());
        const cv::FileNode &fileHandle = _fs[id];
        CHECK_NE(fileHandle.type(), cv::FileNode::NONE) << "Missing parameter: " << id.c_str() << " in file: " << _filePath.c_str();

        const cv::FileNode &fileHandle2 = fileHandle[id2];
        CHECK_NE(fileHandle2.type(), cv::FileNode::NONE)
            << "Missing nested parameter: " << id2.c_str() << " inside "
            << id.c_str() << '\n'
            << " in file: " << _filePath.c_str();
        CHECK(fileHandle.isMap())
            << "I think that if this is not a map, we can't use >>";
        fileHandle2 >> *CHECK_NOTNULL(output);
    }

private:
    void openFile(const std::string &filePath, cv::FileStorage *fs) const
    {
        if (filePath.empty())
        {
            LOG(ERROR) << "FilePath empty: " << filePath;
        }
        try
        {
            fs->open(filePath, cv::FileStorage::READ);
        }
        catch (const cv::Exception &e)
        {
            LOG(ERROR) << "Cannot open file: " << filePath << ", OpenCV error code: " << e.what() << '\n';
        }
        LOG_IF(ERROR, !fs->isOpened()) << "Cannot open file in parseYAML: " << filePath
                                       << " (remember that the first line should be: %YAML:1.0)";
    }

    inline void closeFile(cv::FileStorage *fs) const
    {
        fs->release();
    }

    cv::FileStorage _fs;
    std::string _filePath;
};

} // namespace msckf_vio
#endif