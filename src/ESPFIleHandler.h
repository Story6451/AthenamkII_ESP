#pragma once
#include <Arduino.h>
#include <FS.h>

class ESPFileHandler
{
    public:
        void ListDir(fs::FS &fs, const char * dirname, uint8_t levels);
        void CreateDir(fs::FS &fs, const char * path);
        void RemoveDir(fs::FS &fs, const char * path);
        void ReadFile(fs::FS &fs, const char * path);
        void WriteFile(fs::FS &fs, const char * path, const char * message);
        void AppendFile(fs::FS &fs, const char * path, const char * message);
        void RenameFile(fs::FS &fs, const char * path1, const char * path2);
        void DeleteFile(fs::FS &fs, const char * path);
        uint32_t TestFileIO(fs::FS &fs, const char * path);
};