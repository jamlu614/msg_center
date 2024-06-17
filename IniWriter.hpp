#include <string>
#include <map>
#include <fstream>
#include <sstream>

class IniWriter
{
private:
    std::string filename;
    std::map<std::string, std::map<std::string, std::string>> sections;

public:
    IniWriter(const std::string &file) : filename(file)
    {
        parseFileToMap(filename,sections);
    }

    void parseFileToMap(const std::string &filename,
                        std::map<std::string, std::map<std::string, std::string>> &sections)
    {
        std::ifstream file(filename);
        if (!file.is_open())
        {
            std::cerr << "无法打开文件: " << filename << std::endl;
            return;
        }

        std::string currentSection;
        std::string line;

        while (std::getline(file, line))
        {
            // 去除前后空白字符
            line.erase(0, line.find_first_not_of(" \t\r\n"));
            line.erase(line.find_last_not_of(" \t\r\n") + 1);

            // 跳过空行和注释
            if (line.empty() || line[0] == ';' || line[0] == '#')
            {
                continue;
            }

            // 检查是否是节(section)
            if (line.front() == '[' && line.back() == ']')
            {
                currentSection = line.substr(1, line.size() - 2);
                // 确保节名在map中存在
                sections[currentSection]; // 如果不存在则创建空map
                continue;
            }

            // 解析键值对
            size_t delimiterPos = line.find('=');
            if (delimiterPos != std::string::npos)
            {
                std::string key = line.substr(0, delimiterPos);
                std::string value = line.substr(delimiterPos + 1);

                // 去除键和值的前后空白
                key.erase(0, key.find_first_not_of(" \t\r\n"));
                key.erase(key.find_last_not_of(" \t\r\n") + 1);
                value.erase(0, value.find_first_not_of(" \t\r\n"));
                value.erase(value.find_last_not_of(" \t\r\n") + 1);

                // 如果没有当前节，使用空字符串作为默认节
                if (currentSection.empty())
                {
                    sections[""][key] = value;
                }
                else
                {
                    sections[currentSection][key] = value;
                }
            }
        }

        file.close();
    }

    // 添加或修改配置项
    void set(const std::string &section, const std::string &key, const std::string &value)
    {
        sections[section][key] = value;
    }

    // 添加数字值
    template <typename T>
    void set(const std::string &section, const std::string &key, T value)
    {
        sections[section][key] = std::to_string(value);
    }

    // 保存到文件
    bool save()
    {
        std::ofstream file(filename);
        if (!file.is_open())
            return false;

        for (const auto &section : sections)
        {
            // 写入section标题
            if (!section.first.empty()) {
                file << "[" << section.first << "]\n";
            }

            // 写入该section下的所有键值对
            for (const auto &keyValue : section.second)
            {
                file << keyValue.first << "=" << keyValue.second << "\n";
            }

            file << "\n"; // section之间空一行
        }

        file.close();
        return true;
    }
};