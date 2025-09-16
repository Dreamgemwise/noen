/* ftdata.cpp  --------------------------------------------------------------
 *  Sniff EtherCAT F/T frame and publish as geometry_msgs/WrenchStamped
 *  2025-07-⋯
 * -------------------------------------------------------------------------*/



/**
 * sudo setcap cap_net_raw,cap_net_admin+ep /home/triyi/catkin_ws/devel/lib/ftdata/ftdata
 * rosrun ftdata ftdata
*/
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

#include <pcap/pcap.h>
#include <netinet/ether.h>
#include <arpa/inet.h>

#include <cstring>
#include <sstream>
#include <iomanip>

/* -------------------- 工具 ------------------------------------------------ */
static const uint16_t ETH_P_ECAT = 0x88A4;          // EtherCAT Ether-Type

inline float le_u32_to_float(const uint8_t *p)
{
  uint32_t v =  p[0] |
              (p[1] <<  8) |
              (p[2] << 16) |
              (p[3] << 24);
  float f; std::memcpy(&f, &v, 4);
  return f;
}

std::string hex_dump(const uint8_t *d, size_t n)
{
  std::ostringstream oss;
  oss << std::hex << std::setfill('0');
  for (size_t i = 0; i < n; ++i) oss << std::setw(2) << unsigned(d[i]) << ' ';
  return oss.str();
}

/* -------------------- 入口 ------------------------------------------------ */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ftdata_sniffer");
  ros::NodeHandle   nh;
  ros::NodeHandle pnh("~");                    // 私有参数

  std::string iface = "enp4s0";
  int          ft_off = 0x04b4;               // **从帧首字节计数** 的偏移
  bool         debug  = false;

  pnh.param("iface",   iface,   iface);
  pnh.param("ft_off",  ft_off,  ft_off);
  pnh.param("debug",   debug,   debug);

  ROS_INFO_STREAM("Sniffing EtherCAT on " << iface
                  << ", absolute FT_OFF=0x" << std::hex << ft_off);

  /* ---- 打开 pcap 设备 ---- */
  char errbuf[PCAP_ERRBUF_SIZE]{};
  pcap_t *handle = pcap_open_live(iface.c_str(), 65535, 1, 1, errbuf);
  if (!handle)
  { ROS_FATAL_STREAM("pcap_open_live: " << errbuf); return 1; }

  /* Ether-Type 过滤器 */
  struct bpf_program fp{};
  std::ostringstream flt;  flt << "ether proto 0x" << std::hex << ETH_P_ECAT;
  if (pcap_compile(handle, &fp, flt.str().c_str(), 1, PCAP_NETMASK_UNKNOWN) < 0 ||
      pcap_setfilter(handle, &fp) < 0)
  { ROS_FATAL("pcap filter error"); return 1; }

  ros::Publisher pub =
      nh.advertise<geometry_msgs::WrenchStamped>("ft_data_raw", 1000);

  ros::Rate loop(1000);                       // 1 kHz 足够
  while (ros::ok())
  {
    const uint8_t      *pkt;
    struct pcap_pkthdr *hdr;
    int ret = pcap_next_ex(handle, &hdr, &pkt);   // 1=ok 0=timeout -1/-2 err/EOF
    if (ret <= 0) { loop.sleep(); continue; }

    if (hdr->caplen < static_cast<size_t>(ft_off + 24))  // 不够长，丢
      continue;

    const uint8_t *ft_ptr = pkt + ft_off;

    if (debug)
      ROS_INFO_STREAM_THROTTLE(1.0,
        "HEX[" << std::dec << ft_off << "] " << hex_dump(ft_ptr, 24));

    /* 解析 6 × float32 (小端) */
    float Fx = le_u32_to_float(ft_ptr +  0);
    float Fy = le_u32_to_float(ft_ptr +  4);
    float Fz = le_u32_to_float(ft_ptr +  8);
    float Tx = le_u32_to_float(ft_ptr + 12);
    float Ty = le_u32_to_float(ft_ptr + 16);
    float Tz = le_u32_to_float(ft_ptr + 20);

    geometry_msgs::WrenchStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.wrench.force.x  = Fx;
    msg.wrench.force.y  = Fy;
    msg.wrench.force.z  = Fz;
    msg.wrench.torque.x = Tx;
    msg.wrench.torque.y = Ty;
    msg.wrench.torque.z = Tz;
    pub.publish(msg);
  }
  pcap_close(handle);
  return 0;
}
















// #include <ros/ros.h>
// #include <geometry_msgs/WrenchStamped.h>

// #include <pcap/pcap.h>
// #include <arpa/inet.h>
// #include <netinet/ether.h>
// #include <cstring>
// #include <sstream>
// #include <vector>
// #include <iomanip>

// /* ------------------------------- 常量与工具 ------------------------------ */

// static const uint16_t ETH_P_ECAT = 0x88A4; // EtherCAT 以太网类型

// /** 将 4 字节按照小端解读成 float */
// inline float le_u32_to_float(const uint8_t *p)
// {
//   uint32_t v = p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24);
//   float f;
//   std::memcpy(&f, &v, 4);
//   return f;
// }

// /** 打印十六进制串（debug 用） */
// std::string hex_dump(const uint8_t *data, size_t len)
// {
//   std::ostringstream oss;
//   for (size_t i = 0; i < len; ++i)
//   {
//     oss << std::hex << std::setw(2) << std::setfill('0') << uint32_t(data[i]) << ' ';
//   }
//   return oss.str();
// }

// /* ------------------------------- 节点入口 ------------------------------- */

// int main(int argc, char **argv)
// {
//   /* ---- ROS 初始化 ---- */
//   ros::init(argc, argv, "ftdata_sniffer");
//   ros::NodeHandle nh;
//   ros::NodeHandle pnh("~"); // 私有参数空间

//   /* ---- 读取参数 ---- */
//   std::string iface = "enp4s0";
//   int ft_off = 0x04b4; // 起始偏移（字节）
//   bool debug = true;

//   pnh.param("iface", iface, iface);
//   pnh.param("ft_off", ft_off, ft_off);
//   pnh.param("debug", debug, debug);

//   ROS_INFO_STREAM("Sniffing EtherCAT on " << iface << ", FT_OFF=0x" << std::hex << ft_off);

//   /* ---- 打开 pcap 设备 ---- */
//   char errbuf[PCAP_ERRBUF_SIZE]{};
//   pcap_t *handle = pcap_open_live(iface.c_str(), 65535, 1, 1, errbuf);
//   if (!handle)
//   {
//     ROS_FATAL_STREAM("pcap_open_live failed: " << errbuf);
//     return 1;
//   }

//   /* 仅抓 EtherCAT */
//   struct bpf_program fp{};
//   std::ostringstream filter;
//   filter << "ether proto 0x" << std::hex << ETH_P_ECAT;
//   if (pcap_compile(handle, &fp, filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN) < 0 || pcap_setfilter(handle, &fp) < 0)
//   {
//     ROS_FATAL("pcap filter error");
//     return 1;
//   }

//   /* ---- 发布者 ---- */
//   ros::Publisher pub = nh.advertise<geometry_msgs::WrenchStamped>("ft_data_raw", 1000);

//   /* ---- 主循环 ---- */
//   ros::Rate r(1000); // 1 kHz 足够
//   while (ros::ok())
//   {
//     struct pcap_pkthdr *hdr;
//     const uint8_t *pkt;
//     int ret = pcap_next_ex(handle, &hdr, &pkt);
//     if (ret <= 0)
//     {
//       r.sleep();
//       continue;
//     } // -1 error, 0 timeout

//     /* EtherCAT frame payload 紧随 14 字节以太网头 */
//     const uint8_t *payload = pkt + 14;

//     /* 确保偏移足够 */
//     if (static_cast<int>(hdr->caplen) < 14 + ft_off + 24)
//       continue;

//     const uint8_t *ft_ptr = payload + ft_off;

//     if (debug)
//     {
//       ROS_INFO_STREAM_THROTTLE(1.0,
//                                "HEX[" << std::dec << ft_off << "] " << hex_dump(ft_ptr, 24));
//     }

//     /* 解析 6 × float32 (Fx Fy Fz Tx Ty Tz) — 小端 */
//     float Fx = le_u32_to_float(ft_ptr + 0);
//     float Fy = le_u32_to_float(ft_ptr + 4);
//     float Fz = le_u32_to_float(ft_ptr + 8);
//     float Tx = le_u32_to_float(ft_ptr + 12);
//     float Ty = le_u32_to_float(ft_ptr + 16);
//     float Tz = le_u32_to_float(ft_ptr + 20);

//     geometry_msgs::WrenchStamped msg;
//     msg.header.stamp = ros::Time::now();
//     msg.wrench.force.x = Fx;
//     msg.wrench.force.y = Fy;
//     msg.wrench.force.z = Fz;
//     msg.wrench.torque.x = Tx;
//     msg.wrench.torque.y = Ty;
//     msg.wrench.torque.z = Tz;
//     pub.publish(msg);
//   }

//   pcap_close(handle);
//   return 0;
// }

// // ftdata_sniffer.cpp – print raw EtherCAT bytes around FT sensor for debugging
// // Build: catkin_make; run with _iface:=enp4s0  _ft_off:=0x04b4  _debug:=true
// #include <ros/ros.h>
// #include <std_msgs/Header.h>
// #include <geometry_msgs/WrenchStamped.h>
// #include <net/ethernet.h>
// #include <netpacket/packet.h>
// #include <net/if.h>
// #include <sys/socket.h>
// #include <arpa/inet.h>
// #include <linux/if_ether.h>
// #include <unistd.h>
// #include <fcntl.h>
// #include <sys/ioctl.h>
// #include <endian.h>
// #include <iomanip>
// #include <sstream>
// #include <cstring>

// struct Config
// {
//   std::string iface = "enp4s0";
//   uint32_t FT_OFF = 0x04b4;           // byte offset of first FT value in Rx frame
//   bool data_is_int16 = true;          // false = float32 little‑endian
//   double scale_force = 500.0 / 32768; // N/LSB if int16, else 1.0 when float
//   double scale_torque = 20.0 / 32768; // N·m/LSB
//   bool debug_hex = false;             // dump raw bytes once per packet
// } cfg;

// // helper to hex‑dump n bytes starting at ptr
// static std::string hexDump(const uint8_t *p, size_t n)
// {
//   std::ostringstream ss;
//   ss << std::hex << std::setfill('0');
//   for (size_t i = 0; i < n; ++i)
//   {
//     ss << std::setw(2) << (int)p[i];
//     if (i + 1 < n)
//       ss << " ";
//   }
//   return ss.str();
// }

// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "ftdata_sniffer");
//   ros::NodeHandle nh("~");
//   // param overrides
//   nh.param("iface", cfg.iface, cfg.iface);
//   int tmp_off;
//   nh.param("ft_off", tmp_off, (int)cfg.FT_OFF);
//   cfg.FT_OFF = static_cast<uint32_t>(tmp_off);
//   bool tmp_int16;
//   nh.param("int16", tmp_int16, cfg.data_is_int16);
//   cfg.data_is_int16 = tmp_int16;
//   nh.param("scale_force", cfg.scale_force, cfg.scale_force);
//   nh.param("scale_torque", cfg.scale_torque, cfg.scale_torque);
//   nh.param("debug", cfg.debug_hex, cfg.debug_hex);

//   int sock = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
//   if (sock < 0)
//   {
//     perror("socket");
//     return 1;
//   }
//   // bind to interface
//   sockaddr_ll sll{};
//   sll.sll_family = AF_PACKET;
//   sll.sll_protocol = htons(ETH_P_ALL);
//   sll.sll_ifindex = if_nametoindex(cfg.iface.c_str());
//   if (bind(sock, (sockaddr *)&sll, sizeof(sll)) < 0)
//   {
//     perror("bind");
//     return 1;
//   }
//   ROS_INFO("Sniffing EtherCAT on %s, FT_OFF=0x%04x", cfg.iface.c_str(), cfg.FT_OFF);

//   ros::Publisher pub = nh.advertise<geometry_msgs::WrenchStamped>("wrench", 1000);
//   uint8_t buf[1600];
//   while (ros::ok())
//   {
//     int n = recv(sock, buf, sizeof(buf), 0);
//     if (n <= (int)(cfg.FT_OFF + 24))
//       continue; // need at least 24 bytes for 6 values
//     // simple filter: EtherType 0x88A4 (EtherCAT)
//     uint16_t ethertype = ntohs(*(uint16_t *)(buf + 12));
//     if (ethertype != 0x88A4)
//       continue;

//     const uint8_t *ptr = buf + cfg.FT_OFF;
//     if (cfg.debug_hex)
//     {
//       // dump 32 bytes around payload
//       ROS_INFO_STREAM_THROTTLE(1, "HEX[" << std::dec << cfg.FT_OFF << "] " << hexDump(ptr, 32));
//     }

//     geometry_msgs::WrenchStamped msg;
//     msg.header.stamp = ros::Time::now();

//     if (cfg.data_is_int16)
//     {
//       int16_t raw[6];
//       for (int i = 0; i < 6; ++i)
//         raw[i] = le16toh(*(int16_t *)(ptr + 2 * i));
//       msg.wrench.force.x = raw[0] * cfg.scale_force;
//       msg.wrench.force.y = raw[1] * cfg.scale_force;
//       msg.wrench.force.z = raw[2] * cfg.scale_force;
//       msg.wrench.torque.x = raw[3] * cfg.scale_torque;
//       msg.wrench.torque.y = raw[4] * cfg.scale_torque;
//       msg.wrench.torque.z = raw[5] * cfg.scale_torque;
//     }
//     else
//     {
//       float f[6];
//       memcpy(f, ptr, 6 * sizeof(float));
//       msg.wrench.force.x = f[0];
//       msg.wrench.force.y = f[1];
//       msg.wrench.force.z = f[2];
//       msg.wrench.torque.x = f[3];
//       msg.wrench.torque.y = f[4];
//       msg.wrench.torque.z = f[5];
//     }
//     pub.publish(msg);
//   }
//   return 0;
// }

// // ftdata_sniffer.cpp — passive EtherCAT sniffer to extract 6‑axis F/T data from slave‑5 and publish as /ft_sensor
// // ------------------------------------------------------------------------------------------------------------------
// //  * DOES NOT act as an EtherCAT master; it only listens on RAW socket (libpcap) in promiscuous mode.
// //  * Decodes the 24‑byte TxPDO of slave‑5 located at FT_OFF bytes from EtherCAT frame start.
// //  * Supports both 6×float and 6×int16 output formats (select via DATA_IS_INT16).
// //  * Publishes geometry_msgs/WrenchStamped at the bus cycle rate (≈1 kHz).
// //
// // Build (inside catkin_ws):
// //   catkin_create_pkg ftdata_sniffer roscpp geometry_msgs libpcap
// //   # copy this file into src/
// //   catkin_make -DCMAKE_BUILD_TYPE=Release
// //   sudo setcap cap_net_raw,cap_net_admin+ep devel/lib/ftdata_sniffer/ftdata_sniffer
// //
// // Run:
// //   rosrun ftdata_sniffer ftdata_sniffer _iface:=enp4s0 _ft_off:=0x0A34 _int16:=true
// //
// #include <pcap.h>
// #include <ros/ros.h>
// #include <geometry_msgs/WrenchStamped.h>
// #include <signal.h>
// #include <cstring>
// #include <arpa/inet.h>

// static pcap_t *handle = nullptr;
// static bool running = true;

// //---------------------------------------------------------------------------
// struct Config
// {
//   std::string iface = "enp4s0";       // NIC name
//   uint32_t FT_OFF = 0x04b4;           // byte offset of first F/T value
//   bool data_is_int16 = true;          // true=int16, false=float
//   // double scale_force = 500.0 / 32768; // N/LSB when int16
//   // double scale_torque = 20.0 / 32768; // N·m/LSB when int16
//   double scale_force = 0.001; // N/LSB when int16
//   double scale_torque = 0.001; // N·m/LSB when int16

// } cfg;

// //---------------------------------------------------------------------------
// void sigint(int)
// {
//   running = false;
//   if (handle)
//     pcap_breakloop(handle);
// }

// inline void decode_and_publish(const uint8_t *frame, size_t len, ros::Publisher &pub)
// {
//   if (len < cfg.FT_OFF + (cfg.data_is_int16 ? 12 : 24))
//     return; // sanity

//   geometry_msgs::WrenchStamped msg;
//   msg.header.stamp = ros::Time::now();

//   if (cfg.data_is_int16)
//   {
//     const int16_t *raw = reinterpret_cast<const int16_t *>(frame + cfg.FT_OFF);
//     msg.wrench.force.x = raw[0] * cfg.scale_force;
//     msg.wrench.force.y = raw[1] * cfg.scale_force;
//     msg.wrench.force.z = raw[2] * cfg.scale_force;
//     msg.wrench.torque.x = raw[3] * cfg.scale_torque;
//     msg.wrench.torque.y = raw[4] * cfg.scale_torque;
//     msg.wrench.torque.z = raw[5] * cfg.scale_torque;
//   }
//   else
//   {
//     const float *ft = reinterpret_cast<const float *>(frame + cfg.FT_OFF);
//     msg.wrench.force.x = ft[0];
//     msg.wrench.force.y = ft[1];
//     msg.wrench.force.z = ft[2];
//     msg.wrench.torque.x = ft[3];
//     msg.wrench.torque.y = ft[4];
//     msg.wrench.torque.z = ft[5];
//   }
//   pub.publish(msg);
// }

// // pcap callback ----------------------------------------------------------------
// void packet_cb(u_char *arg, const struct pcap_pkthdr *hdr, const u_char *bytes)
// {
//   ros::Publisher *pub = reinterpret_cast<ros::Publisher *>(arg);
//   // strip Ethernet header (14 B) → EtherCAT datagram starts at bytes+14
//   const uint8_t *ec = bytes + 14;
//   size_t ec_len = hdr->caplen - 14;
//   decode_and_publish(ec, ec_len, *pub);
// }

// //---------------------------------------------------------------------------
// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "ftdata_sniffer");
//   ros::NodeHandle nh("~");

//   nh.param("iface", cfg.iface, cfg.iface);
//   int tmp_off;
//   nh.param("ft_off", tmp_off, static_cast<int>(cfg.FT_OFF));
//   cfg.FT_OFF = static_cast<uint32_t>(tmp_off);
//   bool tmp_int16;
//   nh.param("int16", tmp_int16, cfg.data_is_int16);
//   cfg.data_is_int16 = tmp_int16;
//   nh.param("scale_force", cfg.scale_force, cfg.scale_force);
//   nh.param("scale_torque", cfg.scale_torque, cfg.scale_torque);

//   char errbuf[PCAP_ERRBUF_SIZE] = {0};
//   handle = pcap_open_live(cfg.iface.c_str(), 2048, 1, 1, errbuf);
//   if (!handle)
//   {
//     ROS_FATAL("pcap open failed: %s", errbuf);
//     return 1;
//   }

//   // filter ether proto 0x88a4
//   struct bpf_program prog;
//   if (pcap_compile(handle, &prog, "ether proto 0x88a4", 1, PCAP_NETMASK_UNKNOWN) ||
//       pcap_setfilter(handle, &prog))
//   {
//     ROS_FATAL("pcap filter error");
//     pcap_close(handle);
//     return 1;
//   }

//   ros::Publisher pub = nh.advertise<geometry_msgs::WrenchStamped>("/ft_sensor", 10);
//   ROS_INFO("Sniffing EtherCAT on %s, FT_OFF=0x%X (%u) bytes, format=%s", cfg.iface.c_str(), cfg.FT_OFF, cfg.FT_OFF, cfg.data_is_int16 ? "INT16" : "FLOAT");

//   signal(SIGINT, sigint);

//   while (running && ros::ok())
//   {
//     int r = pcap_dispatch(handle, 10, packet_cb, reinterpret_cast<u_char *>(&pub));
//     if (r < 0)
//     {
//       ROS_WARN("pcap_dispatch error %d", r);
//       break;
//     }
//     ros::spinOnce();
//   }
//   pcap_close(handle);
//   return 0;
// }

// // ftdata_sniffer.cpp — passive EtherCAT sniffer to extract 6‑axis F/T data from slave‑5 and publish as /ft_sensor
// // ------------------------------------------------------------------------------------------------------------------
// //  * DOES NOT act as an EtherCAT master; it only listens on RAW socket (libpcap) in promiscuous mode.
// //  * Decodes the 24‑byte TxPDO of slave‑5 located at FT_OFF bytes from EtherCAT frame start.
// //  * Supports both 6×float and 6×int16 output formats (select via DATA_IS_INT16).
// //  * Publishes geometry_msgs/WrenchStamped at the bus cycle rate (≈1 kHz).
// //
// // Build (inside catkin_ws):
// //   catkin_create_pkg ftdata_sniffer roscpp geometry_msgs libpcap
// //   # copy this file into src/
// //   catkin_make -DCMAKE_BUILD_TYPE=Release
// //   sudo setcap cap_net_raw,cap_net_admin+ep devel/lib/ftdata_sniffer/ftdata_sniffer
// //
// // Run:
// //   rosrun ftdata_sniffer ftdata_sniffer _iface:=enp4s0 _ft_off:=0x0A34 _int16:=true
// //
// #include <pcap.h>
// #include <ros/ros.h>
// #include <geometry_msgs/WrenchStamped.h>
// #include <signal.h>
// #include <cstring>
// #include <arpa/inet.h>

// static pcap_t* handle = nullptr;
// static bool    running = true;

// //---------------------------------------------------------------------------
// struct Config
// {
//   std::string iface          = "enp4s0";   // NIC name
//   uint32_t    FT_OFF         = 0x0A34;      // byte offset of first F/T value
//   bool        data_is_int16  = true;        // true=int16, false=float
//   double      scale_force    = 500.0/32768; // N/LSB when int16
//   double      scale_torque   =  20.0/32768; // N·m/LSB when int16
// } cfg;

// //---------------------------------------------------------------------------
// void sigint(int){ running = false; if(handle) pcap_breakloop(handle);}

// inline void decode_and_publish(const uint8_t* frame, size_t len, ros::Publisher& pub)
// {
//   if(len < cfg.FT_OFF + (cfg.data_is_int16?12:24)) return; // sanity

//   geometry_msgs::WrenchStamped msg;
//   msg.header.stamp = ros::Time::now();

//   if(cfg.data_is_int16){
//     const int16_t* raw = reinterpret_cast<const int16_t*>(frame + cfg.FT_OFF);
//     msg.wrench.force.x  = raw[0] * cfg.scale_force;
//     msg.wrench.force.y  = raw[1] * cfg.scale_force;
//     msg.wrench.force.z  = raw[2] * cfg.scale_force;
//     msg.wrench.torque.x = raw[3] * cfg.scale_torque;
//     msg.wrench.torque.y = raw[4] * cfg.scale_torque;
//     msg.wrench.torque.z = raw[5] * cfg.scale_torque;
//   }else{
//     const float* ft = reinterpret_cast<const float*>(frame + cfg.FT_OFF);
//     msg.wrench.force.x  = ft[0];
//     msg.wrench.force.y  = ft[1];
//     msg.wrench.force.z  = ft[2];
//     msg.wrench.torque.x = ft[3];
//     msg.wrench.torque.y = ft[4];
//     msg.wrench.torque.z = ft[5];
//   }
//   pub.publish(msg);
// }

// // pcap callback ----------------------------------------------------------------
// void packet_cb(u_char* arg, const struct pcap_pkthdr* hdr, const u_char* bytes)
// {
//   ros::Publisher* pub = reinterpret_cast<ros::Publisher*>(arg);
//   // strip Ethernet header (14 B) → EtherCAT datagram starts at bytes+14
//   const uint8_t* ec = bytes + 14;
//   size_t ec_len = hdr->caplen - 14;
//   decode_and_publish(ec, ec_len, *pub);
// }

// //---------------------------------------------------------------------------
// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "ftdata_sniffer");
//   ros::NodeHandle nh("~");

//   nh.param("iface",        cfg.iface,        cfg.iface);
//   nh.param("ft_off",       cfg.FT_OFF,       cfg.FT_OFF);
//   nh.param("int16",        cfg.data_is_int16,cfg.data_is_int16);
//   nh.param("scale_force",  cfg.scale_force,  cfg.scale_force);
//   nh.param("scale_torque", cfg.scale_torque, cfg.scale_torque);

//   char errbuf[PCAP_ERRBUF_SIZE] = {0};
//   handle = pcap_open_live(cfg.iface.c_str(), 2048, 1, 1, errbuf);
//   if(!handle){ ROS_FATAL("pcap open failed: %s", errbuf); return 1; }

//   // filter ether proto 0x88a4
//   struct bpf_program prog;
//   if(pcap_compile(handle, &prog, "ether proto 0x88a4", 1, PCAP_NETMASK_UNKNOWN) ||
//      pcap_setfilter(handle, &prog))
//   { ROS_FATAL("pcap filter error"); pcap_close(handle); return 1; }

//   ros::Publisher pub = nh.advertise<geometry_msgs::WrenchStamped>("/ft_sensor", 10);
//   ROS_INFO("Sniffing EtherCAT on %s, FT_OFF=0x%X (%u) bytes, format=%s", cfg.iface.c_str(), cfg.FT_OFF, cfg.FT_OFF, cfg.data_is_int16?"INT16":"FLOAT");

//   signal(SIGINT, sigint);

//   while(running && ros::ok()){
//     int r = pcap_dispatch(handle, 10, packet_cb, reinterpret_cast<u_char*>(&pub));
//     if(r < 0){ ROS_WARN("pcap_dispatch error %d", r); break; }
//     ros::spinOnce();
//   }
//   pcap_close(handle);
//   return 0;
// }

// // ftdata_sniffer.cpp — passive EtherCAT sniffer that extracts 6‑axis F/T data from slave‑5 PDO
// // -----------------------------------------------------------------------------------------------------------------
// //  * DOES **NOT** act as an EtherCAT master; it only opens a RAW socket in promiscuous mode (ETH_P_ALL), captures
// //    frames whose Ether‑Type == 0x88A4 (EtherCAT), and decodes the 24‑byte TxPDO of slave‑5 located at a fixed
// //    offset (FT_OFF bytes from start of EtherCAT payload).
// //  * Publishes geometry_msgs/WrenchStamped to topic ~/wrench (ROS1 Melodic).
// //  * Needs CAP_NET_RAW capability or root.
// //  * Adjust FT_OFF (byte offset of first float Fx in the process‑data image) if your mapping changes.
// // -----------------------------------------------------------------------------------------------------------------
// //  Build (inside catkin package ftdata):
// //      add_executable(ftdata_sniffer src/ftdata_sniffer.cpp)
// //      target_link_libraries(ftdata_sniffer ${catkin_LIBRARIES} pcap)
// //  Run:
// //      sudo setcap cap_net_raw+ep $(rospack find ftdata)/devel/lib/ftdata/ftdata_sniffer
// //      rosrun ftdata ftdata_sniffer _iface:=enp4s0 _ft_off:=1204
// // -----------------------------------------------------------------------------------------------------------------

// #include <ros/ros.h>
// #include <geometry_msgs/WrenchStamped.h>
// #include <pcap/pcap.h>
// #include <net/ethernet.h>
// #include <netinet/in.h>
// #include <cstring>
// #include <cstdlib>
// #include <csignal>

// static volatile bool run = true;
// void sigint(int){ run = false; }

// // helper: convert 4‑byte little‑endian float
// static inline float le_float(const uint8_t *p){ float f; std::memcpy(&f, p, 4); return f; }

// int main(int argc,char** argv)
// {
//   ros::init(argc,argv,"ftdata_sniffer");
//   ros::NodeHandle nh("~");

//   std::string iface="enp4s0"; nh.getParam("iface", iface);
//   int ft_off = 1204;         nh.getParam("ft_off", ft_off);   // byte offset of Fx in PDO image

//   geometry_msgs::WrenchStamped msg;
//   ros::Publisher pub = nh.advertise<geometry_msgs::WrenchStamped>("wrench",10);

//   char errbuf[PCAP_ERRBUF_SIZE];
//   pcap_t *pcap = pcap_open_live(iface.c_str(), 1600, 1/*promisc*/, 1/*ms timeout*/, errbuf);
//   if(!pcap){ ROS_FATAL("pcap_open_live: %s", errbuf); return 1; }
//   // compile filter: ether proto 0x88a4
//   struct bpf_program fp;
//   if(pcap_compile(pcap,&fp,"ether proto 0x88a4",1,PCAP_NETMASK_UNKNOWN)<0 ||
//      pcap_setfilter(pcap,&fp)<0){ ROS_FATAL("pcap filter error"); return 1; }

//   signal(SIGINT,sigint);
//   ROS_INFO("Sniffing EtherCAT on %s, FT_OFF=%d", iface.c_str(), ft_off);

//   while(run && ros::ok()){
//     struct pcap_pkthdr *hdr; const u_char *data;
//     int rc = pcap_next_ex(pcap,&hdr,&data);
//     if(rc==0) continue;          // timeout
//     if(rc<0)  break;             // error / EOF

//     if(hdr->caplen < sizeof(struct ether_header)+ft_off+24) continue;  // not long enough

//     const uint8_t *ecat = data + sizeof(struct ether_header); // EtherCAT payload start (command+idx+…)
//     const uint8_t *ft   = ecat + ft_off;

//     msg.header.stamp = ros::Time::now();
//     msg.wrench.force.x  = le_float(ft+0);
//     msg.wrench.force.y  = le_float(ft+4);
//     msg.wrench.force.z  = le_float(ft+8);
//     msg.wrench.torque.x = le_float(ft+12);
//     msg.wrench.torque.y = le_float(ft+16);
//     msg.wrench.torque.z = le_float(ft+20);
//     pub.publish(msg);

//     ros::spinOnce();
//   }
//   pcap_close(pcap);
//   return 0;
// }

// #include <ros/ros.h>
// #include <geometry_msgs/WrenchStamped.h>
// #include <soem/ethercat.h>
// #include <signal.h>
// #include <errno.h>

// static const int FT_IDX = 5;     // Slave index of the FT sensor (1‑based)
// static const int LOOP_HZ = 500; // Control / publish rate
// static volatile bool g_stop = false;

// void sigintHandler(int) { g_stop = true; }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "ftdata", ros::init_options::NoSigintHandler);
//     ros::NodeHandle nh;
//     ros::NodeHandle pnh("~");

//     signal(SIGINT, sigintHandler);

//     std::string iface;
//     pnh.param<std::string>("iface", iface, "enp4s0");
//     ROS_INFO_STREAM("Using interface: " << iface);

//     // -------------------- 1. initialise SOEM --------------------
//     if (ec_init(iface.c_str()) <= 0)
//     {
//         perror("ec_init");
//         ROS_FATAL("ec_init failed – raw‑socket not permitted? Wrong iface?");
//         return 1;
//     }

//     if (ec_config_init(FALSE) <= 0)
//     {
//         ROS_FATAL("No slaves found – check cabling / power");
//         ec_close();
//         return 1;
//     }

//     if (ec_slavecount < FT_IDX)
//     {
//         ROS_FATAL_STREAM("Expected FT slave at index " << FT_IDX << " but only "
//                                                        << ec_slavecount << " slaves present");
//         ec_close();
//         return 1;
//     }

//     // Distributed Clocks optional
//     ec_configdc();
//     // Map all PDOs to local memory
//     int pd_sz = ec_config_map(nullptr);
//     if (pd_sz <= 0)
//     {
//         ROS_FATAL("PDO mapping failed");
//         ec_close();
//         return 1;
//     }

//     // Request all slaves to OP
//     ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
//     ec_slave[0].state = EC_STATE_OPERATIONAL;
//     ec_writestate(0);
//     int chk = 40;
//     do
//     {
//         ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
//     } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

//     if (ec_slave[0].state != EC_STATE_OPERATIONAL)
//     {
//         ROS_FATAL("Failed to reach OP state");
//         ec_close();
//         return 1;
//     }

//     const uint8_t *pd = ec_slave[FT_IDX].inputs;
//     if (pd == nullptr) {
//         ROS_FATAL("ec_slave[FT_IDX].inputs is nullptr");
//         ec_close();
//         return 1;
//     }
//     const uint32_t ft_offset = 0x04b4;
//     ROS_INFO_STREAM("FT process‑data offset = " << ft_offset << " bytes");

//     // 检查内存区域是否足够大
//     if (pd + ft_offset + 6 * sizeof(float) > pd + ec_slave[FT_IDX].Ibytes) {
//         ROS_FATAL("Insufficient memory for reading FT data");
//         ec_close();
//         return 1;
//     }

//     ros::Publisher pub = nh.advertise<geometry_msgs::WrenchStamped>("/ft_sensor", 10);
//     ros::Rate rate(LOOP_HZ);

//     // -------------------- 2. main loop --------------------
//     while (ros::ok() && !g_stop)
//     {
//         ec_send_processdata();
//         int wkc = ec_receive_processdata(EC_TIMEOUTRET);
//         if (wkc <= 0)
//         {
//             ROS_WARN_THROTTLE(1.0, "No processdata – WKC = %d", wkc);
//             rate.sleep();
//             continue;
//         }

//         const int32_t *i = reinterpret_cast<const int32_t *>(pd + ft_offset);
//         geometry_msgs::WrenchStamped msg;
//         msg.header.stamp = ros::Time::now();
//         msg.wrench.force.x = static_cast<float>(i[0]) * 0.001;
//         msg.wrench.force.y = static_cast<float>(i[1]) * 0.001;
//         msg.wrench.force.z = static_cast<float>(i[2]) * 0.001;
//         msg.wrench.torque.x = static_cast<float>(i[3]) * 0.001;
//         msg.wrench.torque.y = static_cast<float>(i[4]) * 0.001;
//         msg.wrench.torque.z = static_cast<float>(i[5]) * 0.001;
//         pub.publish(msg);

//         ros::spinOnce();
//         rate.sleep();
//     }

//     // -------------------- 3. shutdown --------------------
//     ec_slave[0].state = EC_STATE_INIT;
//     ec_writestate(0);
//     ec_close();
//     ROS_INFO("FT node stopped cleanly");
//     return 0;
// }

// // ftdata.cpp – read 6‑axis F/T sensor over SOEM and publish as geometry_msgs/WrenchStamped
// // ------------------------------------------------------------------
// // • expects an EtherCAT F/T sensor as slave FT_IDX on the bus
// // • obtains the interface name from the private ROS param "iface" (default "enp4s0")
// // • dynamically determines process‑data offset via ec_slave[FT_IDX].Istart
// // • checks all error paths to avoid segfaults
// // • requires cap_net_raw & cap_net_admin or root privileges
// // ------------------------------------------------------------------

// #include <ros/ros.h>
// #include <geometry_msgs/WrenchStamped.h>
// #include <soem/ethercat.h>
// #include <signal.h>
// #include <errno.h>

// static const int FT_IDX = 5;     // Slave index of the FT sensor (1‑based)
// static const int LOOP_HZ = 1000; // Control / publish rate
// static volatile bool g_stop = false;

// void sigintHandler(int) { g_stop = true; }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "ftdata", ros::init_options::NoSigintHandler);
//     ros::NodeHandle nh;
//     ros::NodeHandle pnh("~");

//     signal(SIGINT, sigintHandler);

//     std::string iface;
//     pnh.param<std::string>("iface", iface, "enp4s0");
//     ROS_INFO_STREAM("Using interface: " << iface);

//     // -------------------- 1. initialise SOEM --------------------
//     if (ec_init(iface.c_str()) <= 0)
//     {
//         perror("ec_init");
//         ROS_FATAL("ec_init failed – raw‑socket not permitted? Wrong iface?");
//         return 1;
//     }

//     if (ec_config_init(FALSE) <= 0)
//     {
//         ROS_FATAL("No slaves found – check cabling / power");
//         ec_close();
//         return 1;
//     }

//     if (ec_slavecount < FT_IDX)
//     {
//         ROS_FATAL_STREAM("Expected FT slave at index " << FT_IDX << " but only "
//                                                        << ec_slavecount << " slaves present");
//         ec_close();
//         return 1;
//     }

//     // Distributed Clocks optional
//     ec_configdc();
//     // Map all PDOs to local memory
//     int pd_sz = ec_config_map(nullptr);
//     if (pd_sz <= 0)
//     {
//         ROS_FATAL("PDO mapping failed");
//         ec_close();
//         return 1;
//     }

//     // Request all slaves to OP
//     ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
//     ec_slave[0].state = EC_STATE_OPERATIONAL;
//     ec_writestate(0);
//     int chk = 40;
//     do
//     {
//         ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
//     } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

//     if (ec_slave[0].state != EC_STATE_OPERATIONAL)
//     {
//         ROS_FATAL("Failed to reach OP state");
//         ec_close();
//         return 1;
//     }

//     const uint8_t *pd = ec_slave[FT_IDX].inputs;
//     // const uint32_t ft_offset = ec_slave[FT_IDX].Istart;
//     const uint32_t ft_offset = 0x04b4;
//     ROS_INFO_STREAM("FT process‑data offset = " << ft_offset << " bytes");

//     ros::Publisher pub = nh.advertise<geometry_msgs::WrenchStamped>("/ft_sensor", 10);
//     ros::Rate rate(LOOP_HZ);

//     // -------------------- 2. main loop --------------------
//     while (ros::ok() && !g_stop)
//     {
//         ec_send_processdata();
//         int wkc = ec_receive_processdata(EC_TIMEOUTRET);
//         if (wkc <= 0)
//         {
//             ROS_WARN_THROTTLE(1.0, "No processdata – WKC = %d", wkc);
//             rate.sleep();
//             continue;
//         }

//         const float *f = reinterpret_cast<const float *>(pd + ft_offset);
//         geometry_msgs::WrenchStamped msg;
//         msg.header.stamp = ros::Time::now();
//         msg.wrench.force.x = f[0];
//         msg.wrench.force.y = f[1];
//         msg.wrench.force.z = f[2];
//         msg.wrench.torque.x = f[3];
//         msg.wrench.torque.y = f[4];
//         msg.wrench.torque.z = f[5];
//         pub.publish(msg);

//         ros::spinOnce();
//         rate.sleep();
//     }

//     // -------------------- 3. shutdown --------------------
//     ec_slave[0].state = EC_STATE_INIT;
//     ec_writestate(0);
//     ec_close();
//     ROS_INFO("FT node stopped cleanly");
//     return 0;
// }

// // soem_ft_node.cpp
// #include <ros/ros.h>
// #include <geometry_msgs/WrenchStamped.h>
// #include "ethercat.h"

// static const char *IFACE = "enp4s0";
// static const int FT_IDX = 5;   // Wireshark 找到的从站序号 (1 基)
// static const int OFF = 0x04b4; // 首字节偏移
// static const int LOOP_HZ = 500;

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "ftdata");
//     ros::NodeHandle nh;
//     ros::Publisher pub = nh.advertise<geometry_msgs::WrenchStamped>("ft_sensor", 10);

//     int ret = ec_init(IFACE); // 直接传递 IFACE
//     if (ret <= 0)
//     {
//         perror("ec_init");
//         ROS_FATAL("ec_init failed");
//         return -1; // 立即退出，而不是继续往下
//     }
//     if (ec_config_init(FALSE) <= 0)
//     {
//         ROS_FATAL("No slaves found – exiting");
//         ec_close();
//         return -1;
//     }

//     ec_config_map(nullptr);

//     ros::Rate r(LOOP_HZ);
//     while (ros::ok())
//     {
//         ec_send_processdata();
//         int wkc = ec_receive_processdata(EC_TIMEOUTRET);
//         if (wkc > 0)
//         {
//             uint8_t *d = ec_slave[FT_IDX].inputs + OFF;
//             int32_t fx = *reinterpret_cast<int32_t *>(d + 0);
//             int32_t fy = *reinterpret_cast<int32_t *>(d + 4);
//             int32_t fz = *reinterpret_cast<int32_t *>(d + 8);
//             int32_t tx = *reinterpret_cast<int32_t *>(d + 12);
//             int32_t ty = *reinterpret_cast<int32_t *>(d + 16);
//             int32_t tz = *reinterpret_cast<int32_t *>(d + 20);

//             geometry_msgs::WrenchStamped m;
//             m.header.stamp = ros::Time::now();
//             m.wrench.force.x = fx * 0.001; // 根据缩放调整
//             m.wrench.force.y = fy * 0.001;
//             m.wrench.force.z = fz * 0.001;
//             m.wrench.torque.x = tx * 0.001;
//             m.wrench.torque.y = ty * 0.001;
//             m.wrench.torque.z = tz * 0.001;
//             pub.publish(m);
//         }
//         r.sleep();
//     }
//     ec_close();
//     return 0;
// }

// // soem_ft_node.cpp
// #include <ros/ros.h>
// #include <geometry_msgs/WrenchStamped.h>
// #include "ethercat.h"

// static const char *IFACE = "enp4s0";
// static const int FT_IDX = 5;   // Wireshark 找到的从站序号 (1 基)
// static const int OFF = 0x0224; // 首字节偏移
// static const int LOOP_HZ = 500;

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "ftdata");
//     ros::NodeHandle nh;
//     ros::Publisher pub = nh.advertise<geometry_msgs::WrenchStamped>("ft_sensor", 10);

//     // if (ec_init(IFACE) <= 0 || ec_config_init(FALSE) <= 0)
//     // {
//     //     perror("ec_init"); // 打印系统 errno
//     //     ROS_FATAL("SOEM init fail");
//     //     return -1;
//     // }
//     int ret = ec_init(IFACE.c_str());
//     if (ret <= 0)
//     {
//         perror("ec_init");
//         ROS_FATAL("ec_init failed");
//         return -1; // 立即退出，而不是继续往下
//     }
//     if (ec_config_init(FALSE) <= 0)
//     {
//         ROS_FATAL("No slaves found – exiting");
//         ec_close();
//         return -1;
//     }

//     ec_config_map(nullptr);

//     ros::Rate r(LOOP_HZ);
//     while (ros::ok())
//     {
//         ec_send_processdata();
//         int wkc = ec_receive_processdata(EC_TIMEOUTRET);
//         if (wkc > 0)
//         {
//             uint8_t *d = ec_slave[FT_IDX].inputs + OFF;
//             int32_t fx = *reinterpret_cast<int32_t *>(d + 0);
//             int32_t fy = *reinterpret_cast<int32_t *>(d + 4);
//             int32_t fz = *reinterpret_cast<int32_t *>(d + 8);
//             int32_t tx = *reinterpret_cast<int32_t *>(d + 12);
//             int32_t ty = *reinterpret_cast<int32_t *>(d + 16);
//             int32_t tz = *reinterpret_cast<int32_t *>(d + 20);

//             geometry_msgs::WrenchStamped m;
//             m.header.stamp = ros::Time::now();
//             m.wrench.force.x = fx * 0.001; // 根据缩放调整
//             m.wrench.force.y = fy * 0.001;
//             m.wrench.force.z = fz * 0.001;
//             m.wrench.torque.x = tx * 0.001;
//             m.wrench.torque.y = ty * 0.001;
//             m.wrench.torque.z = tz * 0.001;
//             pub.publish(m);
//         }
//         r.sleep();
//     }
//     ec_close();
//     return 0;
// }
