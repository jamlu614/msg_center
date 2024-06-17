#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <chrono>
#include <nlohmann/json.hpp>
#include <signal.h>
#include <array>
#include <thread>
#include <string>
#include <iomanip>
#include <sstream>
#include <gac_ipc_metro.hpp>
#include <mosquitto.h>
#include <charconv>
// #define INI_USE_STACK 0
// #define INI_ALLOW_REALLOC 4096
// #define INI_MAX_LINE 4096
#include <ini.h>
#include <libgen.h> // for dirname
#include <linux/limits.h>
#include <fstream>
#include <fcntl.h>
#include <regex>
#include <geometry_msgs.pb.h>
#include <Eigen/Eigen>
#include "IniWriter.hpp"
#include <filesystem>
#include "Approbot.pb.h"

using json = nlohmann::json;
using namespace std;

typedef struct
{
    int version;
    int onLine;
    string lineCode;
    string stationCode;
    string entranceCode;
    string checkpointCode;
    string equipmentModel;
    string equipmentId;
    string licence;
} Configuration;

// #define SERVER_IP "58.248.160.145"   // 服务器 IP（本地测试用 127.0.0.1）
// #define PORT 31356                   // 服务器端口
#define SERVER_IP "172.23.64.42"     // 服务器 IP（本地测试用 127.0.0.1）
#define PORT 9031                    // 服务器端口
#define BUFFER_SIZE 1024 * 1024 * 16 // 缓冲区大小
int64_t interval_ms = 1000 * 5;      // 5秒发送一次
// #define LICENCE "3a5a8327a382ebb2598502f3737d11731ba5961761905329148ca1aa9ecc0e4ecc00494f63e31d526df599e9fdfb012dcfbae98dfd0cbd4f4ecb1dfeffa9e285850ab495c6be82ad2be7f6f73c21bbd7cbc3ae6a4eff9253b24b46ddff12b6540c31caf85c8b68ae289076cbd1f6e46bd6625976b5058d30ac96e2938fac83a7b9c20ce063796dd54c1a0a169eb96c"

string lineCode = "04";
string stationCode = "05";
string entranceCode = "H0";
string checkpointCode = "1";
string equipmentModel = "anjian";
string version = "1.0.0";

int clientSocket;
std::thread heartbeat_thread;
int32_t message_seq = 1;
std::string equipmentId; // 设备id
int32_t eventNum = 1;
int32_t eventPost = -1; // app岗位id
int32_t onLine = 0;
bool exitFlag = false;
Configuration config;
std::atomic<float> rotation(0);
std::atomic<float> x(0), y(0), z(0);

struct mosquitto *mosq;

int sendPacket(unsigned char message_type[2], string payload);
std::string get_event_up_payload(int32_t event_type, char *imageData);
static int handler_INI(void *user, const char *section, const char *name,
                       const char *value)
{
    Configuration *pconfig = (Configuration *)user;
#define MATCH(s, n) strcmp(section, s) == 0 && strcmp(name, n) == 0
    if (MATCH("user", "version"))
    {
        pconfig->version = atoi(value);
    }
    else if (MATCH("user", "onLine"))
    {
        pconfig->onLine = atoi(value);
    }
    else if (MATCH("user", "lineCode"))
    {
        pconfig->lineCode = strdup(value);
    }
    else if (MATCH("user", "stationCode"))
    {
        pconfig->stationCode = strdup(value);
    }
    else if (MATCH("user", "entranceCode"))
    {
        pconfig->entranceCode = strdup(value);
    }
    else if (MATCH("user", "checkpointCode"))
    {
        pconfig->checkpointCode = strdup(value);
    }
    else if (MATCH("user", "equipmentModel"))
    {
        pconfig->equipmentModel = strdup(value);
    }
    else if (MATCH("user", "equipmentId"))
    {
        pconfig->equipmentId = strdup(value);
    }
    else if (MATCH("user", "licence"))
    {
        pconfig->licence = strdup(value);
    }
    else if (MATCH("user", "licence2"))
    {
        pconfig->licence += strdup(value);
    }
    else
    {
        return 0; /* unknown section/name, error */
    }
    return 1;
}

std::string get_executable_path()
{
    char buf[PATH_MAX];
    ssize_t len = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (len != -1)
    {
        buf[len] = '\0';
        return std::string(buf);
    }
    else
    {
        perror("readlink");
        return "";
    }
}

std::string get_executable_directory()
{
    std::string path = get_executable_path();
    if (!path.empty())
    {
        char dir[PATH_MAX];
        // 使用 dirname
        char *result = dirname(dir); // 注意：dirname 会修改 dir 的内容
        if (result != NULL)
        {
            return std::string(result);
        }
    }
    return "";
}

Eigen::Vector3d quat2eularRPY(double w, double x, double y, double z)
{
    Eigen::Quaterniond q(w, x, y, z);
    q.normalize();

    // 转换为矩阵形式
    Eigen::Matrix3d R = q.toRotationMatrix();

    // 从旋转矩阵提取欧拉角 (ZYX顺序)
    Eigen::Vector3d euler_zyx = R.eulerAngles(2, 1, 0); // ZYX顺序对应2,1,0

    // 返回欧拉角 [roll, pitch, yaw]
    return Eigen::Vector3d(euler_zyx(2), euler_zyx(1), euler_zyx(0));
}

void message_callback(struct mosquitto *mosq, void *obj, const struct mosquitto_message *msg)
{
    //  std::cout << "Received: " << msg->topic << " -> " << (char *)msg->payload << std::endl;
    std::cout << "online:" << onLine << std::endl;
    std::cout << "Received: " << msg->topic << std::endl;
    if (!onLine)
    {
        if (strcmp(msg->topic, "rtc-app/msg") == 0 && !exitFlag)
        {
            GacIpcMetro data;
            data.algorithm_type = 101;
            gac_ipc_metro_write(data);
            std::cout << "发送检测结果到执行端" << std::endl;
        }
        return;
    }
    if (strcmp(msg->topic, "rtc-app/msg") == 0 && !equipmentId.empty() && !exitFlag)
    {
        auto json_str = get_event_up_payload(0, (char *)msg->payload);
        unsigned char message_type[2] = {0x09, 0x20};
        int ret = sendPacket(message_type, json_str);
        std::cout << "上报事件" << std::endl;
        GacIpcMetro data;
        data.algorithm_type = 101;
        gac_ipc_metro_write(data);
    }
    else if (strcmp(msg->topic, "device/robot") == 0 && !exitFlag)
    {
        device::AppRobot data;
        data.ParseFromArray(msg->payload, msg->payloadlen);
        if (data.robot_character() == 1)
        {
            eventPost = 1;
        }
        else if (data.robot_character() == 3)
        {
            eventPost = 2;
        }
        else
        {
            eventPost = -1;
        }
    }
    else if (strcmp(msg->topic, "robot_pose_hq") == 0 && !exitFlag)
    {
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.ParseFromString((char *)msg->payload);
        const auto pos = poseStamped.pose().position();
        const auto quat = poseStamped.pose().orientation();

        Eigen::Vector3d euler = quat2eularRPY(quat.w(), quat.x(), quat.y(), quat.z());
        double degrees = euler(2) * (180.0 / M_PI);
        rotation.store(degrees);
        x.store(pos.x());
        y.store(pos.y());
        z.store(pos.z());
        cout << "收到定位" << endl;
        cout << "rotation:" << rotation.load() << endl;
        cout << "x:" << x.load() << endl;
        cout << "y:" << y.load() << endl;
        cout << "z:" << z.load() << endl;
    }
    else if (strcmp(msg->topic, "nav/robot_pose_hq") == 0 && !exitFlag)
    {
        json payload = json::parse((char *)msg->payload);
        json quat = payload["pose"]["orientation"];
        json position = payload["pose"]["position"];
        x.store(position["x"]);
        x.store(position["y"]);
        x.store(position["z"]);
        Eigen::Vector3d euler = quat2eularRPY(quat["w"], quat["x"], quat["y"], quat["z"]);
        double degrees = euler(2) * (180.0 / M_PI);
        rotation.store(degrees);
        cout << "收到定位" << endl;
        cout << "rotation:" << rotation.load() << endl;
        cout << "x:" << x.load() << endl;
        cout << "y:" << y.load() << endl;
        cout << "z:" << z.load() << endl;
    }
}

void connect_callback(struct mosquitto *mosq, void *obj, int result)
{
    if (!result)
    {
        // mosquitto_subscribe(mosq, NULL, "rtc-app/msg", 1);
        mosquitto_subscribe(mosq, NULL, "device/robot", 1);
        mosquitto_subscribe(mosq, NULL, "robot_pose_hq", 1);
        mosquitto_subscribe(mosq, NULL, "nav/robot_pose_hq", 1);
    }
}

void init_mqtt()
{
    mosquitto_lib_init();
    mosq = mosquitto_new("msg-center", true, nullptr);
    mosquitto_connect_callback_set(mosq, connect_callback);
    mosquitto_message_callback_set(mosq, message_callback);

    if (mosquitto_connect(mosq, "localhost", 1883, 60) != MOSQ_ERR_SUCCESS)
    {
        std::cerr << "Failed to connect!" << std::endl;
        return;
    }
    mosquitto_loop_start(mosq);
    std::cout << "mqtt init" << std::endl;
}

int64_t getSimplifiedTimestamp()
{
    auto now = std::chrono::system_clock::now();
    auto unix_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                              now.time_since_epoch())
                              .count(); // 毫秒级时间戳（13 位）

    std::cout << "Timestamp (milliseconds): " << unix_timestamp << std::endl;
    // printf("Timestamp (milliseconds): %ld\n", unix_timestamp);
    return unix_timestamp;
}

static void Handler(int sig)
{
    exitFlag = true;
    fprintf(stderr, "\nCaught Exiting...\n");
}

std::array<unsigned char, 26> get_pkg_head(unsigned char message_type[2], int32_t payload_len)
{
    int32_t total_length = 26 + payload_len;
    // std::cout << "get_pkg_head body_length:" << payload_len << std::endl;
    // std::cout << "get_pkg_head total_length:" << total_length << std::endl;
    std::array<unsigned char, 26> packet_head{};
    unsigned char start_flag[4] = {0x23, 0x23, 0x23, 0x23};
    int offset = 0;
    std::memcpy(packet_head.data(), start_flag, 4);
    offset += 4;
    std::memcpy(packet_head.data() + offset, &total_length, 4);
    offset += 4;
    std::memcpy(packet_head.data() + offset, &message_seq, 4);
    offset += 4;
    std::memcpy(packet_head.data() + offset, message_type, 2);
    offset += 2;
    std::memcpy(packet_head.data() + offset, &payload_len, 4);
    offset += 4;

    int64_t timestamp = getSimplifiedTimestamp();
    std::memcpy(packet_head.data() + offset, &timestamp, 8);

    static_assert(packet_head.size() == 26, "Packet head size mismatch!");
    std::cout << "message_seq:" << message_seq << std::endl;
    // for (int i = 0; i < 26; ++i)
    // {
    //     std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0')
    //               << static_cast<int>(static_cast<unsigned char>(packet_head[i])) << " ";
    // }
    // std::cout << std::dec;
    return packet_head;
}

std::array<unsigned char, 26> get_response_pkg_head(unsigned char message_type[2], int32_t payload_len, int32_t seq)
{
    int32_t total_length = 26 + payload_len;
    std::array<unsigned char, 26> packet_head{};
    unsigned char start_flag[4] = {0x23, 0x23, 0x23, 0x23};
    int offset = 0;
    std::memcpy(packet_head.data(), start_flag, 4);
    offset += 4;
    std::memcpy(packet_head.data() + offset, &total_length, 4);
    offset += 4;
    std::memcpy(packet_head.data() + offset, &seq, 4);
    offset += 4;
    std::memcpy(packet_head.data() + offset, message_type, 2);
    offset += 2;
    std::memcpy(packet_head.data() + offset, &payload_len, 4);
    offset += 4;

    int64_t timestamp = getSimplifiedTimestamp();
    std::memcpy(packet_head.data() + offset, &timestamp, 8);

    static_assert(packet_head.size() == 26, "Packet head size mismatch!");

    return packet_head;
}

std::string getMacAddress(const std::string &interface)
{
    if (exitFlag)
    {
        return "";
    }
    std::string macPath = "/sys/class/net/" + interface + "/address";
    std::ifstream macFile(macPath);
    if (!macFile.is_open())
    {
        return "";
    }

    std::string macAddress;
    std::getline(macFile, macAddress);
    macFile.close();

    return macAddress;
}

/**
 * 设备认证参数
 */
std::string get_device_authentication_payload()
{
    json payload;
    payload["equipmentId"] = "";
    if (config.equipmentId != "-1")
        payload["equipmentId"] = config.equipmentId;
    payload["licence"] = config.licence;

    payload["equipmentSn"] = getMacAddress("p2p0");
    payload["equipmentModel"] = equipmentModel;
    payload["equipmentType"] = "20";
    payload["lineCode"] = lineCode;
    payload["stationCode"] = stationCode;
    payload["entranceCode"] = entranceCode;
    payload["checkpointCode"] = checkpointCode;
    payload["version"] = "1.0.0";
    std::string json_str = payload.dump();
    return json_str;
}

std::string get_heartbeat_payload()
{
    json payload;
    payload["equipmentId"] = equipmentId;
    std::string json_str = payload.dump();

    return json_str;
}

std::string get_event_down_payload()
{
    json payload;
    payload["code"] = 1;
    payload["message"] = "";
    std::string json_str = payload.dump();
    return json_str;
}

std::string get_event_up_payload(int32_t event_type, char *imageData)
{
    json payload;
    payload["eventType"] = event_type;
    payload["severity"] = 0;
    // 获取当前时间的时间戳
    std::time_t now = std::time(nullptr);
    // 转换为本地时间结构体
    std::tm *local_tm = std::localtime(&now);
    std::ostringstream oss;
    oss << std::setw(6) << std::setfill('0') << eventNum; // 宽度3，补零
    std::string event_num = oss.str();
    std::string json_str = payload.dump();

    std::string event_id = equipmentId +
                           to_string(local_tm->tm_year) +
                           to_string((local_tm->tm_mon + 1)) +
                           to_string(local_tm->tm_mday) +
                           event_num;
    payload["eventId"] = event_id;

    payload["occurredAt"] = getSimplifiedTimestamp();
    json jsonObj = json::parse(imageData);
    // std::cout << "image_data:" << imageData << std::endl;
    payload["images"] = {jsonObj};
    eventNum += 1;
    return json_str;
}

std::string exec_command(const char *cmd)
{
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe)
        throw std::runtime_error("popen() failed!");
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
    {
        result += buffer.data();
    }
    return result;
}

std::string get_device_status_payload()
{
    std::string ip = "";
    std::string osVersion = "";
    std::string output = exec_command("ip addr show p2p0");
    std::smatch match;
    std::regex ip_regex(R"(inet (\d+\.\d+\.\d+\.\d+))");
    if (std::regex_search(output, match, ip_regex))
    {
        std::cout << "WiFi IP: " << match[1] << std::endl;
        ip = match[1].str().c_str();
    }
    else
    {
        std::cout << "No WiFi IP found." << std::endl;
    }

    try
    {
        // 获取发行版描述（如 "Ubuntu 22.04.3 LTS"）
        osVersion = exec_command("lsb_release -ds");
        osVersion.erase(std::remove(osVersion.begin(), osVersion.end(), '\n'), osVersion.end()); // 去掉换行符
        std::cout << "Description: " << osVersion << std::endl;
    }
    catch (const std::exception &e)
    {
        // std::cerr << "Error: " << e.what() << std::endl;
        std::cout << "获取系统版本出错！" << endl;
    }
    GacIMRobotStatus status;
    gac_ipc_metro_robot_status_read(status);

    json data;
    data["ip"] = ip;
    data["softwareVersion"] = "1.0.0";
    data["licence"] = config.licence;
    data["equipmentSn"] = getMacAddress("p2p0");
    data["equipmentType"] = "20";
    data["equipmentModel"] = equipmentModel;
    data["osVersion"] = osVersion;
    data["errorCode"] = "20000";
    data["batteryLevel"] = to_string(status.charge_level) + "%";
    data["batteryState"] = "2";
    if (status.charging_status)
        data["batteryState"] = "1";
    json point;
    data["srid"] = "1";
    data["x"] = x.load();
    data["x"] = y.load();
    data["x"] = z.load();
    data["coordinate"] = point;
    json payload;
    payload["code"] = 1;
    payload["message"] = "";
    payload["data"] = data;
    std::string json_str = payload.dump();
    return json_str;
}

std::string get_device_param_payload()
{
    json payload;
    payload["equipmentId"] = equipmentId;
    payload["workMode"] = 1;
    std::string json_str = payload.dump();
    return json_str;
}

int32_t charArrayToInt32(const char bytes[4], bool isBigEndian = true)
{
    int32_t result = 0;
    if (isBigEndian)
    {
        // 大端序：bytes[0] 是最高字节
        result = (bytes[0] << 24) | ((bytes[1] & 0xFF) << 16) |
                 ((bytes[2] & 0xFF) << 8) | (bytes[3] & 0xFF);
    }
    else
    {
        // 小端序：bytes[0] 是最低字节
        result = (bytes[3] << 24) | ((bytes[2] & 0xFF) << 16) |
                 ((bytes[1] & 0xFF) << 8) | (bytes[0] & 0xFF);
    }
    return result;
}

int16_t charArrayToInt16(const char bytes[2], bool isBigEndian = true)
{
    if (isBigEndian)
    {
        // 大端序：bytes[0] 是高位字节
        return (bytes[0] << 8) | (bytes[1] & 0xFF);
    }
    else
    {
        // 小端序：bytes[0] 是低位字节
        return (bytes[1] << 8) | (bytes[0] & 0xFF);
    }
}

int sendPacket(unsigned char message_type[2], string payload)
{
    std::cout << "send json:\n"
              << payload << std::endl;
    int32_t body_length = payload.length();
    int32_t total_length = 26 + body_length;
    // 获取报文头部
    auto packet_head = get_pkg_head(message_type, body_length);
    // 组合报文并发送
    unsigned char pkg[total_length];
    memset(pkg, 0, sizeof(pkg));
    memcpy(pkg, packet_head.data(), 26);
    memcpy(pkg + 26, payload.data(), body_length);
    message_seq += 2;
    // 4. 发送数据
    return send(clientSocket, pkg, total_length, 0);
}

void sendheartbeat()
{
    while (!exitFlag)
    {
        if (equipmentId.empty())
        {
            std::cout << "设备号为空，等待" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
            continue;
        }
        // 获取心跳包参数
        auto json_str = get_heartbeat_payload();
        // 报文消息类型
        unsigned char message_type[2] = {0x00, 0x02};
        sendPacket(message_type, json_str);
        std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
    }
}

bool setBufferSize(int sockfd, int type, int size)
{
    if (setsockopt(sockfd, SOL_SOCKET, type, &size, sizeof(size)) == -1)
    {
        perror("setsockopt failed");
        return false;
    }
    return true;
}

void printBufferSize(int sockfd, int type)
{
    int buffer_size;
    socklen_t len = sizeof(buffer_size);
    if (getsockopt(sockfd, SOL_SOCKET, type, &buffer_size, &len) == -1)
    {
        perror("getsockopt failed");
        return;
    }
    std::cout << (type == SO_RCVBUF ? "Receive" : "Send")
              << " buffer size: " << buffer_size << " bytes" << std::endl;
}

bool readFull(int sockfd, void *buffer, size_t length)
{
    size_t total_read = 0;
    while (total_read < length && !exitFlag)
    {
        ssize_t n = recv(sockfd, (char *)buffer + total_read, length - total_read, 0);
        // 断开连接
        if (n == 0)
        {
            exitFlag = true;
            return false;
        }
        if (n < 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        total_read += n;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return true;
}

// 在字节数组中查找字符串的起始索引
int findStringIndex(const unsigned char *data, size_t dataLength,
                    const char *targetStr, size_t strLength)
{
    if (strLength == 0 || dataLength < strLength)
        return -1;

    for (size_t i = 0; i <= dataLength - strLength; ++i)
    {
        if (memcmp(&data[i], targetStr, strLength) == 0)
        {
            return static_cast<int>(i); // 找到匹配
        }
    }
    return -1; // 未找到
}

void printHexArray(const unsigned char *arr, size_t size)
{
    for (size_t i = 0; i < size; ++i)
    {
        // 每个字节占2位，不足补零
        std::cout << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<int>(arr[i]) << " ";
    }
    std::cout << std::dec << std::endl; // 恢复十进制输出
}

void setNonBlocking(int sockfd)
{
    int flags = fcntl(sockfd, F_GETFL, 0);
    fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);
}

void responsePacket(unsigned char *message_type, int32_t msg_len, int32_t msg_seq, string json_str)
{
    // 获取报文头部
    auto packet_head = get_response_pkg_head(message_type, msg_len - 26, msg_seq);
    // 组合报文并发送
    unsigned char pkg[msg_len];
    memset(pkg, 0, sizeof(pkg));
    memcpy(pkg, packet_head.data(), 26);
    memcpy(pkg + 26, json_str.data(), msg_len - 26);
    send(clientSocket, pkg, msg_len, 0);
}

int main()
{
    cout << "msg_center_version:" << version << endl;
    signal(SIGQUIT, Handler);
    signal(SIGABRT, Handler);
    signal(SIGINT, Handler);

    std::string config_path(get_executable_directory() + "/config.ini");
    std::string config_path2(get_executable_directory() + "/../config.ini");
    if (ini_parse(config_path.c_str(), handler_INI, &config) < 0)
    {
        if (ini_parse(config_path2.c_str(), handler_INI, &config) < 0)
        {
            printf("Can't load 'config.ini'\n");
            return 1;
        }
    }
    onLine = config.onLine;
    lineCode = config.lineCode;
    stationCode = config.stationCode;
    entranceCode = config.entranceCode;
    checkpointCode = config.checkpointCode;
    equipmentModel = config.equipmentModel;
    init_mqtt();
    if (-1 == gac_ipc_metro_init())
    {
        printf("gac_ipc_metro_init error.\n");
        return -1;
    }

    if (onLine)
    {
        // 1. 创建 Socket（IPv4, TCP）
        clientSocket = socket(AF_INET, SOCK_STREAM, 0);

        if (clientSocket == -1)
        {
            std::cerr << "Failed to create socket!" << std::endl;
            return 1;
        }

        // 2. 设置服务器地址
        sockaddr_in serverAddr{};
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = htons(PORT); // 服务器端口

        // 转换 IP 地址（字符串 → 二进制）
        if (inet_pton(AF_INET, SERVER_IP, &serverAddr.sin_addr) <= 0)
        {
            std::cerr << "Invalid address or address not supported!" << std::endl;
            return 1;
        }
        // 设置接收超时（5秒）
        struct timeval tv;
        tv.tv_sec = 5;
        tv.tv_usec = 0;
        setsockopt(clientSocket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        // 设置发送超时（5秒）
        setsockopt(clientSocket, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        int ret = 0;
        // 3. 连接服务器
        ret = connect(clientSocket, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
        while (ret == -1 && !exitFlag)
        {
            std::cerr << "Connection failed!" << std::endl;
            this_thread::sleep_for(std::chrono::seconds(5));
            close(clientSocket);
            ret = connect(clientSocket, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
        }
        cout << "connect ret:" << ret << endl;
        if (ret == -1)
        {
            gac_ipc_metro_exit();
            if (mosq)
            {
                mosquitto_loop_stop(mosq, true);
                mosquitto_disconnect(mosq);
                mosquitto_destroy(mosq);
            }
            mosquitto_lib_cleanup();
            return 0;
        }
        setNonBlocking(clientSocket);
        std::cout << "Connected to server at " << SERVER_IP << ":" << PORT << std::endl;
        // 获取设备认证请求参数
        auto json_str = get_device_authentication_payload();
        unsigned char message_type[2] = {0x00, 0x01};
        ret = sendPacket(message_type, json_str);
        if (ret > 0)
        {
            std::cout << "成功返回" << std::endl;
            std::vector<char> buffer(BUFFER_SIZE * 2);
            const char *start_flag = "####";
            // size_t dataLength = sizeof(buffer.data());
            size_t strLength = strlen(start_flag);
            do
            {
                // 5. 接收服务器响应
                // char buffer[BUFFER_SIZE] = {0};
                buffer.clear();
                memset(buffer.data(), 0, BUFFER_SIZE * 2);
                int bytesRead = recv(clientSocket, buffer.data(), 26, 0);
                int start_index = 0;
                while (bytesRead > 0 && !exitFlag)
                {
                    start_index = findStringIndex((unsigned char *)buffer.data(), bytesRead, start_flag, strLength);

                    std::cout << "bytesRead:" << bytesRead << std::endl;
                    if (start_index == -1)
                    {
                        bytesRead += recv(clientSocket, buffer.data() + bytesRead, 26, 0);
                        this_thread::sleep_for(std::chrono::milliseconds(10));
                        continue;
                    }
                    else
                    {
                        if (bytesRead - start_index >= 26)
                        {
                            std::cout << "Server response: " << buffer.data() << std::endl;

                            // memcpy(frame.data(), buffer.data() + index, bytesRead - index);

                            // 消息序列号
                            int32_t msg_seq = charArrayToInt32(buffer.data() + start_index + 8, false);
                            int16_t msg_type = charArrayToInt16(buffer.data() + start_index + 12, false);
                            int32_t msg_len = charArrayToInt32(buffer.data() + start_index + 4, false);
                            // std::cout << "head:";
                            // for (int i = 0; i < 26; ++i)
                            // {
                            //     std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0')
                            //               << static_cast<int>(static_cast<unsigned char>(buffer[i])) << " ";
                            // }
                            // std::cout << std::endl;
                            char auth16[2] = {0x80, 0x01};
                            char heartbeat16[2] = {0x80, 0x02};
                            char event16[2] = {0x10, 0x20};
                            char status16[2] = {0x00, 0x04};
                            int16_t authType = charArrayToInt16(auth16, false);
                            int16_t heartbeatType = charArrayToInt16(heartbeat16, false);
                            int16_t eventType = charArrayToInt16(event16, false);
                            int16_t statusType = charArrayToInt16(status16, false);
                            char *payload = buffer.data() + start_index + 26;
                            std::cout << "start_index:" << start_index << std::endl;

                            std::cout << "msg_type:" << msg_type << std::endl;
                            std::cout << "authType:" << authType << std::endl;
                            std::cout << "heartbeatType:" << heartbeatType << std::endl;
                            std::cout << "eventType:" << eventType << std::endl;
                            std::cout << "statusType:" << statusType << std::endl;
                            std::cout << "msg_len:" << msg_len << std::endl;
                            // 请求类型
                            // 判断是否有请求序号
                            try
                            {

                                if (!readFull(clientSocket, payload, msg_len - 26))
                                {
                                    std::cerr << "has no payload." << std::endl;
                                    break;
                                }
                                cout << "payload 读取完毕" << endl;
                                printHexArray((unsigned char *)buffer.data(), bytesRead + msg_len - 26);
                                // cout << "payload:" << payload << endl;
                                // int payload_len = start_index + msg_len;
                                // printHexArray((unsigned char *)payload, msg_len - 26);
                                cout << payload << endl;
                                // 解析JSON字符串
                                json json_data = json::parse(payload);
                                std::cout << "Formatted JSON:\n"
                                          << json_data.dump(4) << std::endl;
                                // 答应类型
                                if (msg_type == eventType) // 平台事件下发请求
                                {

                                    // 把事件发送给机器人
                                    // 有效：1=乘客逆行 2=隔栏递物 3=打架斗殴
                                    int32_t eventType = json_data["eventType"];
                                    if ((eventPost == eventType) || eventType == 3)
                                    {
                                        GacIpcMetro data;
                                        data.algorithm_type = eventType;
                                        gac_ipc_metro_write(data);
                                    }

                                    // 响应服务器请求
                                    // 获取事件响应参数
                                    auto json_str = get_event_down_payload();
                                    int32_t body_length = json_str.length();
                                    int32_t total_length = 26 + body_length;
                                    // 报文消息类型
                                    unsigned char message_type[2] = {0x90, 0x20};
                                    responsePacket(message_type, total_length, msg_seq, json_str);
                                }
                                else if (msg_type == authType) // 认证答应
                                {
                                    int32_t code = json_data["code"];
                                    if (code == 1)
                                    {

                                        equipmentId = json_data["data"]["equipmentId"];
                                        std::cout << "设备号:" << equipmentId << std::endl;

                                        std::string config_path(get_executable_directory() + "/config.ini");
                                        std::string config_path2(get_executable_directory() + "/../config.ini");

                                        if (std::filesystem::exists(config_path) && config.equipmentId == "-1")
                                        {
                                            IniWriter writer(config_path);
                                            writer.set("user", "equipmentId", equipmentId);
                                            writer.save();
                                        }
                                        else if (std::filesystem::exists(config_path2) && config.equipmentId == "-1")
                                        {
                                            IniWriter writer(config_path2);
                                            writer.set("user", "equipmentId", equipmentId);
                                            writer.save();
                                        }

                                        if (equipmentId.empty())
                                        {
                                            std::cerr << "认证失败.\n";
                                        }
                                        else
                                        {
                                            std::cout << "认证成功。" << std::endl;
                                            std::cout << "上报设备参数。" << std::endl;
                                            auto json_str = get_device_param_payload();
                                            unsigned char message_type[2] = {0x01, 0x20};
                                            sendPacket(message_type, json_str);
                                            std::cout << "启动心跳线程。" << std::endl;
                                            // 启动心跳线程
                                            heartbeat_thread = std::thread(sendheartbeat);
                                        }
                                    }
                                    else
                                    {
                                        std::cout << "请求认证失败,code:" << code << std::endl;
                                    }
                                }
                                else if (msg_type == heartbeatType) // 心跳答应
                                {
                                    int32_t code = json_data["code"];
                                    if (code == 1)
                                    {
                                        interval_ms = json_data["data"]["interval"];
                                        std::cout << "interval_ms:" << interval_ms << std::endl;
                                    }
                                    else
                                    {
                                        std::string msg = json_data["message"];
                                        std::cerr << "心跳请求失败 msg:" << msg << std::endl;
                                    }
                                }
                                else if (msg_type == statusType) // 设备状态答应
                                {
                                    // 答应设备状态
                                    auto json_str = get_device_status_payload();
                                    unsigned char message_type[2] = {0x80, 0x04};
                                    int32_t total_length = 26 + json_str.length();
                                    responsePacket(message_type, total_length, msg_seq, json_str);
                                }
                            }
                            catch (json::parse_error &e)
                            {
                                std::cerr << "JSON解析错误: " << e.what() << "\n";
                            }
                        }
                        else
                        {
                            bytesRead += recv(clientSocket, buffer.data() + bytesRead, 26 - (bytesRead - start_index), 0);
                            this_thread::sleep_for(std::chrono::milliseconds(10));
                            continue;
                        }
                        this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                    break;
                }
                // 连接断开
                if (bytesRead == 0)
                {
                    // exitFlag = true;
                    continue;
                }
                else
                {
                    // 非阻塞模式下，errno == EAGAIN或EWOULDBLOCK表示无数据
                    if (errno != EAGAIN && errno != EWOULDBLOCK)
                    {
                        // perror("recv error");
                        // break;
                    }
                    this_thread::sleep_for(std::chrono::milliseconds(200));
                }
                // else if (bytesRead == 0)
                // {
                //     // 对方关闭连接
                //     std::cout << "Connection closed by peer." << std::endl;
                //     break;
                // }
                // else
                // {
                //     // 接收错误
                //     perror("recv() failed");
                //     break;
                // }

            } while (!exitFlag);
        }
        std::cout << "exitfalg:" << exitFlag << std::endl;
        heartbeat_thread.join();
        // 6. 关闭 Socket
        close(clientSocket);
    }
    else
    {
        while (!exitFlag)
        {
            sleep(1);
        }
    }
    gac_ipc_metro_exit();
    if (mosq)
    {
        mosquitto_loop_stop(mosq, true);
        mosquitto_disconnect(mosq);
        mosquitto_destroy(mosq);
    }
    mosquitto_lib_cleanup();
    return 0;
}