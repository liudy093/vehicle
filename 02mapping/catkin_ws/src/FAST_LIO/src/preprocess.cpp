#include "preprocess.h"

// 宏定义：返回状态标识
#define RETURN0     0x00    // 返回0
#define RETURN0AND1 0x10    // 返回0和1

/**
 * @brief Preprocess类构造函数
 * 初始化点云预处理相关参数，设置默认值
 */
Preprocess::Preprocess()
  :feature_enabled(0), lidar_type(AVIA), blind(0.01), point_filter_num(1)
{
  inf_bound = 10;           // 无限远边界值（用于过滤远距离点）
  N_SCANS   = 6;            // 扫描线数量
  SCAN_RATE = 10;           // 扫描频率(Hz)
  group_size = 8;           // 点云分组大小（用于特征提取）
  disA = 0.01;              // 距离计算参数A
  disA = 0.1;               // 距离计算参数B（原代码存在笔误，应为disB）
  p2l_ratio = 225;          // 点到线距离比率阈值
  limit_maxmid =6.25;       // 最大-中间距离比率限制
  limit_midmin =6.25;       // 中间-最小距离比率限制
  limit_maxmin = 3.24;      // 最大-最小距离比率限制
  jump_up_limit = 170.0;    // 跳跃上限角度(度)
  jump_down_limit = 8.0;    // 跳跃下限角度(度)
  cos160 = 160.0;           // 160度的余弦值计算参数
  edgea = 2;                // 边缘判断参数a
  edgeb = 0.1;              // 边缘判断参数b
  smallp_intersect = 172.5; // 小平面相交角度(度)
  smallp_ratio = 1.2;       // 小平面比率阈值
  given_offset_time = false;// 是否提供偏移时间标记

  // 将角度转换为弧度的余弦值（用于后续角度判断）
  jump_up_limit = cos(jump_up_limit/180*M_PI);
  jump_down_limit = cos(jump_down_limit/180*M_PI);
  cos160 = cos(cos160/180*M_PI);
  smallp_intersect = cos(smallp_intersect/180*M_PI);
}

/**
 * @brief Preprocess类析构函数
 * 空实现，无特殊资源需要释放
 */
Preprocess::~Preprocess() {}

/**
 * @brief 设置预处理参数
 * @param feat_en 是否启用特征提取
 * @param lid_type 激光雷达类型
 * @param bld 盲区阈值（小于此距离的点将被过滤）
 * @param pfilt_num 点云下采样数量（每隔多少个点保留一个）
 */
void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num)
{
  feature_enabled = feat_en;
  lidar_type = lid_type;
  blind = bld;
  point_filter_num = pfilt_num;
}

/**
 * @brief 处理Livox AVIA激光雷达数据（CustomMsg类型）
 * @param msg 输入的点云消息
 * @param pcl_out 输出的PCL格式点云
 */
#if FAST_LIO_HAS_LIVOX
void Preprocess::process(const livox_ros_driver2::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{  
  avia_handler(msg);         // 调用AVIA专用处理函数
  *pcl_out = pl_surf;        // 输出处理后的平面点云
}
#endif

/**
 * @brief 处理标准PointCloud2类型的激光雷达数据
 * @param msg 输入的点云消息
 * @param pcl_out 输出的PCL格式点云
 */
void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  // 设置时间单位缩放因子（将不同单位的时间统一为毫秒）
  switch (time_unit)
  {
    case SEC:    // 秒
      time_unit_scale = 1.e3f;
      break;
    case MS:     // 毫秒
      time_unit_scale = 1.f;
      break;
    case US:     // 微秒
      time_unit_scale = 1.e-3f;
      break;
    case NS:     // 纳秒
      time_unit_scale = 1.e-6f;
      break;
    default:
      time_unit_scale = 1.f;
      break;
  }

  // 根据激光雷达类型调用对应的处理函数
  switch (lidar_type)
  {
  case AVIA:
    ROS_ERROR_THROTTLE(1.0, "AVIA lidar_type requires livox_ros_driver2 CustomMsg input");
    pl_surf.clear();
    break;

  case OUST64:    // Ouster 64线激光雷达
    oust64_handler(msg);
    break;

  case VELO16:    // Velodyne 16线激光雷达
    velodyne_handler(msg);
    break;

  case MARSIM:    // 仿真点云
    sim_handler(msg);
    break;
  
  default:
    printf("Error LiDAR Type");  // 未知激光雷达类型
    break;
  }
  *pcl_out = pl_surf;  // 输出处理后的平面点云
}

/**
 * @brief Livox AVIA激光雷达数据处理函数
 * @param msg 输入的CustomMsg类型点云消息
 */
#if FAST_LIO_HAS_LIVOX
void Preprocess::avia_handler(const livox_ros_driver2::CustomMsg::ConstPtr &msg)
{
  // 清空点云容器
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  double t1 = omp_get_wtime();  // 记录开始时间
  int plsize = msg->point_num;  // 获取点云数量

  // 预留内存空间
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  pl_full.resize(plsize);

  // 初始化每条扫描线的点云缓冲区
  for(int i=0; i<N_SCANS; i++)
  {
    pl_buff[i].clear();
    pl_buff[i].reserve(plsize);
  }
  uint valid_num = 0;  // 有效点计数
  
  // 如果启用特征提取
  if (feature_enabled)
  {
    for(uint i=1; i<plsize; i++)
    {
      // 筛选有效点（在扫描线范围内且标签有效）
      if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
      {
        // 填充点云数据（坐标、反射率、时间戳）
        pl_full[i].x = msg->points[i].x;
        pl_full[i].y = msg->points[i].y;
        pl_full[i].z = msg->points[i].z;
        pl_full[i].intensity = msg->points[i].reflectivity;
        pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // 曲率字段用作时间戳(毫秒)

        // 判断是否为新点（与前一点坐标不同）
        bool is_new = false;
        if((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7) 
            || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
            || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7))
        {
          // 将新点添加到对应扫描线的缓冲区
          pl_buff[msg->points[i].line].push_back(pl_full[i]);
        }
      }
    }

    // 统计特征提取时间
    static int count = 0;
    static double time = 0.0;
    count ++;
    double t0 = omp_get_wtime();
    
    // 对每条扫描线进行特征提取
    for(int j=0; j<N_SCANS; j++)
    {
      if(pl_buff[j].size() <= 5) continue;  // 点数量过少则跳过
      
      pcl::PointCloud<PointType> &pl = pl_buff[j];  // 当前扫描线点云
      plsize = pl.size();
      vector<orgtype> &types = typess[j];           // 点类型容器
      types.clear();
      types.resize(plsize);
      plsize--;
      
      // 计算每个点的水平距离和与下一点的空间距离
      for(uint i=0; i<plsize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);  // 水平距离
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = sqrt(vx * vx + vy * vy + vz * vz);  // 与下一点的空间距离
      }
      // 最后一个点的水平距离
      types[plsize].range = sqrt(pl[plsize].x * pl[plsize].x + pl[plsize].y * pl[plsize].y);
      
      give_feature(pl, types);  // 提取特征
    }
    
    time += omp_get_wtime() - t0;
    printf("特征提取时间: %lf \n", time / count);  // 输出平均特征提取时间
  }
  // 不启用特征提取时，仅进行点云过滤和下采样
  else
  {
    for(uint i=1; i<plsize; i++)
    {
      // 筛选有效点
      if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
      {
        valid_num ++;
        // 下采样：每隔point_filter_num个点保留一个
        if (valid_num % point_filter_num == 0)
        {
          // 填充点云数据
          pl_full[i].x = msg->points[i].x;
          pl_full[i].y = msg->points[i].y;
          pl_full[i].z = msg->points[i].z;
          pl_full[i].intensity = msg->points[i].reflectivity;
          pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // 曲率字段用作时间戳(毫秒)

          // 过滤近点（小于盲区阈值）和重复点（与前一点坐标相同）
          if(((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7) 
              || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
              || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7))
              && (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z > (blind * blind)))
          {
            pl_surf.push_back(pl_full[i]);
          }
        }
      }
    }
  }
}
#endif

/**
 * @brief Ouster 64线激光雷达数据处理函数
 * @param msg 输入的PointCloud2类型点云消息
 */
void Preprocess::oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  // 清空点云容器
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  
  // 将ROS消息转换为Ouster点云格式
  pcl::PointCloud<ouster_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();
  
  // 预留内存空间
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  
  // 如果启用特征提取
  if (feature_enabled)
  {
    // 初始化每条扫描线的点云缓冲区
    for (int i = 0; i < N_SCANS; i++)
    {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    // 遍历所有点，筛选有效点并按扫描线分类
    for (uint i = 0; i < plsize; i++)
    {
      // 计算距离平方，过滤近点（小于盲区阈值）
      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
      if (range < (blind * blind)) continue;
      
      // 转换点格式（适配自定义PointType）
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      
      // 计算偏航角（用于辅助分类，单位：度）
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.3;  // 弧度转度
      if (yaw_angle >= 180.0)
        yaw_angle -= 360.0;
      if (yaw_angle <= -180.0)
        yaw_angle += 360.0;

      // 设置时间戳（曲率字段，单位：毫秒）
      added_pt.curvature = pl_orig.points[i].t * time_unit_scale;
      
      // 将点添加到对应扫描线的缓冲区
      if(pl_orig.points[i].ring < N_SCANS)
      {
        pl_buff[pl_orig.points[i].ring].push_back(added_pt);
      }
    }

    // 对每条扫描线进行特征提取
    for (int j = 0; j < N_SCANS; j++)
    {
      PointCloudXYZI &pl = pl_buff[j];  // 当前扫描线点云
      int linesize = pl.size();
      vector<orgtype> &types = typess[j];  // 点类型容器
      types.clear();
      types.resize(linesize);
      linesize--;
      
      // 计算每个点的水平距离和与下一点的距离平方
      for (uint i = 0; i < linesize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);  // 水平距离
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;  // 距离平方（避免开方运算，提高效率）
      }
      // 最后一个点的水平距离
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      give_feature(pl, types);  // 提取特征
    }
  }
  // 不启用特征提取时，仅进行点云过滤和下采样
  else
  {
    double time_stamp = msg->header.stamp.toSec();  // 获取时间戳（秒）
    
    for (int i = 0; i < pl_orig.points.size(); i++)
    {
      // 下采样：每隔point_filter_num个点保留一个
      if (i % point_filter_num != 0) continue;

      // 过滤近点（小于盲区阈值）
      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
      if (range < (blind * blind)) continue;
      
      // 转换点格式并填充数据
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.curvature = pl_orig.points[i].t * time_unit_scale;  // 时间戳（毫秒）

      pl_surf.points.push_back(added_pt);
    }
  }
}

/**
 * @brief Velodyne 16线激光雷达数据处理函数
 * @param msg 输入的PointCloud2类型点云消息
 */
void Preprocess::velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    auto process_cloud = [&](const auto &pl_orig, auto read_raw_time, auto convert_to_offset_ms) {
      int plsize = pl_orig.points.size();
      if (plsize == 0) return;

      pl_surf.reserve(plsize);
      const double first_point_time = read_raw_time(pl_orig.points[0]);

      double omega_l = 0.361 * SCAN_RATE;
      std::vector<bool> is_first(N_SCANS, true);
      std::vector<double> yaw_fp(N_SCANS, 0.0);
      std::vector<float> yaw_last(N_SCANS, 0.0);
      std::vector<float> time_last(N_SCANS, 0.0);

      if (read_raw_time(pl_orig.points[plsize - 1]) > first_point_time)
      {
        given_offset_time = true;
      }
      else
      {
        given_offset_time = false;
        double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
        double yaw_end  = yaw_first;
        int layer_first = pl_orig.points[0].ring;
        for (uint i = plsize - 1; i > 0; i--)
        {
          if (pl_orig.points[i].ring == layer_first)
          {
            yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
            break;
          }
        }
      }

      if(feature_enabled)
      {
        for (int i = 0; i < N_SCANS; i++)
        {
          pl_buff[i].clear();
          pl_buff[i].reserve(plsize);
        }

        for (int i = 0; i < plsize; i++)
        {
          PointType added_pt;
          added_pt.normal_x = 0;
          added_pt.normal_y = 0;
          added_pt.normal_z = 0;
          int layer = pl_orig.points[i].ring;
          if (layer >= N_SCANS) continue;

          added_pt.x = pl_orig.points[i].x;
          added_pt.y = pl_orig.points[i].y;
          added_pt.z = pl_orig.points[i].z;
          added_pt.intensity = pl_orig.points[i].intensity;
          added_pt.curvature = convert_to_offset_ms(pl_orig.points[i], first_point_time);

          if (!given_offset_time)
          {
            double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;
            if (is_first[layer])
            {
              yaw_fp[layer] = yaw_angle;
              is_first[layer] = false;
              added_pt.curvature = 0.0;
              yaw_last[layer] = yaw_angle;
              time_last[layer] = added_pt.curvature;
              continue;
            }

            if (yaw_angle <= yaw_fp[layer])
            {
              added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
            }
            else
            {
              added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
            }

            if (added_pt.curvature < time_last[layer])
              added_pt.curvature += 360.0 / omega_l;

            yaw_last[layer] = yaw_angle;
            time_last[layer] = added_pt.curvature;
          }

          pl_buff[layer].points.push_back(added_pt);
        }

        for (int j = 0; j < N_SCANS; j++)
        {
          PointCloudXYZI &pl = pl_buff[j];
          int linesize = pl.size();
          if (linesize < 2) continue;

          vector<orgtype> &types = typess[j];
          types.clear();
          types.resize(linesize);
          linesize--;

          for (uint i = 0; i < linesize; i++)
          {
            types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
            double vx_local = pl[i].x - pl[i + 1].x;
            double vy_local = pl[i].y - pl[i + 1].y;
            double vz_local = pl[i].z - pl[i + 1].z;
            types[i].dista = vx_local * vx_local + vy_local * vy_local + vz_local * vz_local;
          }
          types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
          give_feature(pl, types);
        }
      }
      else
      {
        for (int i = 0; i < plsize; i++)
        {
          PointType added_pt;
          added_pt.normal_x = 0;
          added_pt.normal_y = 0;
          added_pt.normal_z = 0;
          added_pt.x = pl_orig.points[i].x;
          added_pt.y = pl_orig.points[i].y;
          added_pt.z = pl_orig.points[i].z;
          added_pt.intensity = pl_orig.points[i].intensity;
          added_pt.curvature = convert_to_offset_ms(pl_orig.points[i], first_point_time);

          if (!given_offset_time)
          {
            int layer = pl_orig.points[i].ring;
            double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

            if (is_first[layer])
            {
              yaw_fp[layer] = yaw_angle;
              is_first[layer] = false;
              added_pt.curvature = 0.0;
              yaw_last[layer] = yaw_angle;
              time_last[layer] = added_pt.curvature;
              continue;
            }

            if (yaw_angle <= yaw_fp[layer])
            {
              added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
            }
            else
            {
              added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
            }

            if (added_pt.curvature < time_last[layer])
              added_pt.curvature += 360.0 / omega_l;

            yaw_last[layer] = yaw_angle;
            time_last[layer] = added_pt.curvature;
          }

          if (i % point_filter_num == 0)
          {
            if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (blind * blind))
            {
              pl_surf.points.push_back(added_pt);
            }
          }
        }
      }
    };

    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();

    bool has_robosense_timestamp = false;
    for (const auto &field : msg->fields)
    {
      if (field.name == "timestamp")
      {
        has_robosense_timestamp = true;
        break;
      }
    }

    if (has_robosense_timestamp)
    {
      pcl::PointCloud<robosense_ros::Point> pl_orig;
      pcl::fromROSMsg(*msg, pl_orig);
      process_cloud(
        pl_orig,
        [](const robosense_ros::Point &point) { return point.timestamp; },
        [](const robosense_ros::Point &point, double first_point_time) {
          return (point.timestamp - first_point_time) * 1.e3f;
        });
      return;
    }

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    process_cloud(
      pl_orig,
      [](const velodyne_ros::Point &point) { return point.time; },
      [this](const velodyne_ros::Point &point, double) {
        return point.time * time_unit_scale;
      });
}

/**
 * @brief 仿真点云数据处理函数
 * @param msg 输入的PointCloud2类型点云消息
 */
void Preprocess::sim_handler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    // 清空点云容器
    pl_surf.clear();
    pl_full.clear();
    
    // 将ROS消息转换为PCL点云格式
    pcl::PointCloud<pcl::PointXYZI> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.size();
    pl_surf.reserve(plsize);  // 预留内存
    
    // 遍历所有点，过滤近点并转换格式
    for (int i = 0; i < pl_orig.points.size(); i++) {
        // 计算距离平方，过滤近点
        double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                       pl_orig.points[i].z * pl_orig.points[i].z;
        if (range < blind * blind) continue;
        
        // 转换点格式并填充数据
        PointType added_pt;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.curvature = 0.0;  // 仿真点云无时间戳，设为0
        
        pl_surf.points.push_back(added_pt);
    }
}

/**
 * @brief 点云特征提取函数（区分平面点、边缘点等）
 * @param pl 输入的点云
 * @param types 输出的点类型信息
 */
void Preprocess::give_feature(pcl::PointCloud<PointType> &pl, vector<orgtype> &types)
{
  int plsize = pl.size();
  int plsize2;
  if(plsize == 0)
  {
    printf("something wrong\n");  // 点云为空，输出错误信息
    return;
  }
  uint head = 0;  // 有效点起始索引

  // 跳过盲区范围内的点
  while(types[head].range < blind)
  {
    head++;
  }

  // 平面点判断相关变量
  plsize2 = (plsize > group_size) ? (plsize - group_size) : 0;

  Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());  // 当前方向向量
  Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());  // 上一方向向量

  uint i_nex = 0, i2;
  uint last_i = 0; uint last_i_nex = 0;
  int last_state = 0;  // 上一状态（0：非平面，1：平面）
  int plane_type;      // 平面类型

  // 遍历点云，判断平面区域
  for(uint i=head; i<plsize2; i++)
  {
    if(types[i].range < blind)  // 跳过盲区点
    {
      continue;
    }

    i2 = i;

    // 判断当前点是否属于平面
    plane_type = plane_judge(pl, types, i, i_nex, curr_direct);
    
    if(plane_type == 1)  // 属于平面
    {
      // 标记平面内的点（中间点为确定平面点，两端为可能平面点）
      for(uint j=i; j<=i_nex; j++)
      { 
        if(j!=i && j!=i_nex)
        {
          types[j].ftype = Real_Plane;  // 确定的平面点
        }
        else
        {
          types[j].ftype = Poss_Plane;  // 可能的平面点
        }
      }
      
      // 判断与上一平面区域的关系
      if(last_state==1 && last_direct.norm()>0.1)
      {
        double mod = last_direct.transpose() * curr_direct;  // 方向向量点积
        // 方向夹角大于45度且小于135度，标记为平面边缘
        if(mod>-0.707 && mod<0.707)
        {
          types[i].ftype = Edge_Plane;
        }
        else
        {
          types[i].ftype = Real_Plane;  // 方向一致，仍为平面点
        }
      }
      
      i = i_nex - 1;  // 跳过已处理的平面点
      last_state = 1;  // 更新状态为平面
    }
    else  // 非平面
    {
      i = i_nex;       // 移动到下一个待处理点
      last_state = 0;  // 更新状态为非平面
    }

    // 更新上一状态信息
    last_i = i2;
    last_i_nex = i_nex;
    last_direct = curr_direct;
  }

  // 边缘点判断
  plsize2 = plsize > 3 ? plsize - 3 : 0;
  for(uint i=head+3; i<plsize2; i++)
  {
    // 跳过盲区点或已标记为平面的点
    if(types[i].range<blind || types[i].ftype>=Real_Plane)
    {
      continue;
    }

    // 跳过距离过小的点（避免计算错误）
    if(types[i-1].dista<1e-16 || types[i].dista<1e-16)
    {
      continue;
    }

    Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);  // 当前点向量
    Eigen::Vector3d vecs[2];  // 前后点与当前点的向量

    for(int j=0; j<2; j++)
    {
      int m = -1;
      if(j == 1)
      {
        m = 1;  // j=0: 前一个点，j=1: 后一个点
      }

      // 处理前后点为盲区点的情况
      if(types[i+m].range < blind)
      {
        if(types[i].range > inf_bound)
        {
          types[i].edj[j] = Nr_inf;  // 当前点为远点
        }
        else
        {
          types[i].edj[j] = Nr_blind;  // 前后点为盲区点
        }
        continue;
      }

      // 计算向量并归一化
      vecs[j] = Eigen::Vector3d(pl[i+m].x, pl[i+m].y, pl[i+m].z);
      vecs[j] = vecs[j] - vec_a;
      
      // 计算夹角余弦值
      types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();
      
      // 判断角度类型（跳跃点判断）
      if(types[i].angle[j] < jump_up_limit)
      {
        types[i].edj[j] = Nr_180;  // 接近180度（反向）
      }
      else if(types[i].angle[j] > jump_down_limit)
      {
        types[i].edj[j] = Nr_zero;  // 接近0度（同向）
      }
    }

    // 判断是否为边缘跳跃点
    types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();
    if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_zero && types[i].dista>0.0225 && types[i].dista>4*types[i-1].dista)
    {
      if(types[i].intersect > cos160)
      {
        if(edge_jump_judge(pl, types, i, Prev))
        {
          types[i].ftype = Edge_Jump;  // 标记为跳跃边缘点
        }
      }
    }
    else if(types[i].edj[Prev]==Nr_zero && types[i].edj[Next]== Nr_nor && types[i-1].dista>0.0225 && types[i-1].dista>4*types[i].dista)
    {
      if(types[i].intersect > cos160)
      {
        if(edge_jump_judge(pl, types, i, Next))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    else if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_inf)
    {
      if(edge_jump_judge(pl, types, i, Prev))
      {
        types[i].ftype = Edge_Jump;
      }
    }
    else if(types[i].edj[Prev]==Nr_inf && types[i].edj[Next]==Nr_nor)
    {
      if(edge_jump_judge(pl, types, i, Next))
      {
        types[i].ftype = Edge_Jump;
      }
    }
    else if(types[i].edj[Prev]>Nr_nor && types[i].edj[Next]>Nr_nor)
    {
      if(types[i].ftype == Nor)
      {
        types[i].ftype = Wire;  // 标记为线特征
      }
    }
  }

  // 小平面判断
  plsize2 = plsize-1;
  double ratio;
  for(uint i=head+1; i<plsize2; i++)
  {
    // 跳过盲区点
    if(types[i].range<blind || types[i-1].range<blind || types[i+1].range<blind)
    {
      continue;
    }
    
    // 跳过距离过小的点
    if(types[i-1].dista<1e-8 || types[i].dista<1e-8)
    {
      continue;
    }

    // 判断是否为小平面特征
    if(types[i].ftype == Nor)
    {
      // 计算距离比率
      if(types[i-1].dista > types[i].dista)
      {
        ratio = types[i-1].dista / types[i].dista;
      }
      else
      {
        ratio = types[i].dista / types[i-1].dista;
      }

      if(types[i].intersect<smallp_intersect && ratio < smallp_ratio)
      {
        if(types[i-1].ftype == Nor)
        {
          types[i-1].ftype = Real_Plane;
        }
        if(types[i+1].ftype == Nor)
        {
          types[i+1].ftype = Real_Plane;
        }
        types[i].ftype = Real_Plane;
      }
    }
  }

  int last_surface = -1;
  for(uint j=head; j<plsize; j++)
  {
    if(types[j].ftype==Poss_Plane || types[j].ftype==Real_Plane)
    {
      if(last_surface == -1)
      {
        last_surface = j;
      }
    
      if(j == uint(last_surface+point_filter_num-1))
      {
        PointType ap;
        ap.x = pl[j].x;
        ap.y = pl[j].y;
        ap.z = pl[j].z;
        ap.intensity = pl[j].intensity;
        ap.curvature = pl[j].curvature;
        pl_surf.push_back(ap);

        last_surface = -1;
      }
    }
    else
    {
      if(types[j].ftype==Edge_Jump || types[j].ftype==Edge_Plane)
      {
        pl_corn.push_back(pl[j]);
      }
      if(last_surface != -1)
      {
        PointType ap;
        for(uint k=last_surface; k<j; k++)
        {
          ap.x += pl[k].x;
          ap.y += pl[k].y;
          ap.z += pl[k].z;
          ap.intensity += pl[k].intensity;
          ap.curvature += pl[k].curvature;
        }
        ap.x /= (j-last_surface);
        ap.y /= (j-last_surface);
        ap.z /= (j-last_surface);
        ap.intensity /= (j-last_surface);
        ap.curvature /= (j-last_surface);
        pl_surf.push_back(ap);
      }
      last_surface = -1;
    }
  }
}

void Preprocess::pub_func(PointCloudXYZI &pl, const ros::Time &ct)
{
  pl.height = 1; pl.width = pl.size();
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "livox";
  output.header.stamp = ct;
}

int Preprocess::plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct)
{
  double group_dis = disA*types[i_cur].range + disB;
  group_dis = group_dis * group_dis;
  // i_nex = i_cur;

  double two_dis;
  vector<double> disarr;
  disarr.reserve(20);

  for(i_nex=i_cur; i_nex<i_cur+group_size; i_nex++)
  {
    if(types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;
    }
    disarr.push_back(types[i_nex].dista);
  }
  
  for(;;)
  {
    if((i_cur >= pl.size()) || (i_nex >= pl.size())) break;

    if(types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;
    }
    vx = pl[i_nex].x - pl[i_cur].x;
    vy = pl[i_nex].y - pl[i_cur].y;
    vz = pl[i_nex].z - pl[i_cur].z;
    two_dis = vx*vx + vy*vy + vz*vz;
    if(two_dis >= group_dis)
    {
      break;
    }
    disarr.push_back(types[i_nex].dista);
    i_nex++;
  }

  double leng_wid = 0;
  double v1[3], v2[3];
  for(uint j=i_cur+1; j<i_nex; j++)
  {
    if((j >= pl.size()) || (i_cur >= pl.size())) break;
    v1[0] = pl[j].x - pl[i_cur].x;
    v1[1] = pl[j].y - pl[i_cur].y;
    v1[2] = pl[j].z - pl[i_cur].z;

    v2[0] = v1[1]*vz - vy*v1[2];
    v2[1] = v1[2]*vx - v1[0]*vz;
    v2[2] = v1[0]*vy - vx*v1[1];

    double lw = v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2];
    if(lw > leng_wid)
    {
      leng_wid = lw;
    }
  }


  if((two_dis*two_dis/leng_wid) < p2l_ratio)
  {
    curr_direct.setZero();
    return 0;
  }

  uint disarrsize = disarr.size();
  for(uint j=0; j<disarrsize-1; j++)
  {
    for(uint k=j+1; k<disarrsize; k++)
    {
      if(disarr[j] < disarr[k])
      {
        leng_wid = disarr[j];
        disarr[j] = disarr[k];
        disarr[k] = leng_wid;
      }
    }
  }

  if(disarr[disarr.size()-2] < 1e-16)
  {
    curr_direct.setZero();
    return 0;
  }

  if(lidar_type==AVIA)
  {
    double dismax_mid = disarr[0]/disarr[disarrsize/2];
    double dismid_min = disarr[disarrsize/2]/disarr[disarrsize-2];

    if(dismax_mid>=limit_maxmid || dismid_min>=limit_midmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }
  else
  {
    double dismax_min = disarr[0] / disarr[disarrsize-2];
    if(dismax_min >= limit_maxmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }
  
  curr_direct << vx, vy, vz;
  curr_direct.normalize();
  return 1;
}

bool Preprocess::edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir)
{
  if(nor_dir == 0)
  {
    if(types[i-1].range<blind || types[i-2].range<blind)
    {
      return false;
    }
  }
  else if(nor_dir == 1)
  {
    if(types[i+1].range<blind || types[i+2].range<blind)
    {
      return false;
    }
  }
  double d1 = types[i+nor_dir-1].dista;
  double d2 = types[i+3*nor_dir-2].dista;
  double d;

  if(d1<d2)
  {
    d = d1;
    d1 = d2;
    d2 = d;
  }

  d1 = sqrt(d1);
  d2 = sqrt(d2);

 
  if(d1>edgea*d2 || (d1-d2)>edgeb)
  {
    return false;
  }
  
  return true;
}
