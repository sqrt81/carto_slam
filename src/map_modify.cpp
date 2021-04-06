/**
* 在本程序中，我们利用Cartographer使用的非线性优化工具——Ceres，来调整子图的相对位姿。
* Cartographer的建图多少会有一些偏差，但是单个子图内部的误差是相对较小的，子图之间的相对位姿之差会更大。
* 假如我们现在通过人工手段，确定了最后一张子图相对第一张子图的位姿，
* 那我们就可以利用Ceres，在对子图位姿的改变量尽可能小的前提下，确定其余子图之间的位姿关系。
**/

#include <cartographer/io/proto_stream_deserializer.h>
#include <cartographer/io/proto_stream.h>
#include <cartographer/mapping/pose_graph_interface.h>

// Cartographer中用于描述坐标变换的头文件
#include "cartographer/transform/transform.h"

// Ceres求解器
#include <ceres/ceres.h>

#include <ros/ros.h>

using namespace cartographer;
typedef mapping::PoseGraphInterface::Constraint::Pose PoseConstraint;

namespace
{

// Ceres使用自动求导来得到梯度，因此在提供损失函数的时候，只要这样给一个模板就行了。
// 在这个函数中，start和end是两个坐标系的二维坐标和朝向角
// 而relative_pose表示他们的期望相对位姿。
template <typename T>
static std::array<T, 3> ComputeUnscaledError(
    const transform::Rigid2d& relative_pose, const T* const start,
    const T* const end) {
  const T cos_theta_i = cos(start[2]);
  const T sin_theta_i = sin(start[2]);
  const T delta_x = end[0] - start[0];
  const T delta_y = end[1] - start[1];
  const T h[3] = {cos_theta_i * delta_x + sin_theta_i * delta_y,
                  -sin_theta_i * delta_x + cos_theta_i * delta_y,
                  end[2] - start[2]};
  return {{T(relative_pose.translation().x()) - h[0],
           T(relative_pose.translation().y()) - h[1],
           common::NormalizeAngleDifference(
               T(relative_pose.rotation().angle()) - h[2])}};
}

// 这个函数对上面的函数做了一个加权
template <typename T>
std::array<T, 3> ScaleError(const std::array<T, 3>& error,
                            double translation_weight, double rotation_weight)
{
  // clang-format off
  return {{
      error[0] * translation_weight,
      error[1] * translation_weight,
      error[2] * rotation_weight
  }};
  // clang-format on
}

// 对上述函数做一个封装
class SpaCostFunction2D {
public:
    explicit SpaCostFunction2D(
            const PoseConstraint& observed_relative_pose)
        : rel_pose_cons_(observed_relative_pose) {}

    template <typename T>
    bool operator()(const T* const start_pose, const T* const end_pose,
                    T* e) const {
        const std::array<T, 3> error =
                ScaleError(ComputeUnscaledError(
                               transform::Project2D(rel_pose_cons_.zbar_ij),
                               start_pose, end_pose),
                           rel_pose_cons_.translation_weight,
                           rel_pose_cons_.rotation_weight);
        std::copy(std::begin(error), std::end(error), e);
        return true;
    }

private:
    const PoseConstraint rel_pose_cons_;
};

// 可以看到，Ceres的CostFunction是对原始损失函数的自动求导（AutoDiff）封装。
ceres::CostFunction* CreateAutoDiffSpaCostFunction(
    const PoseConstraint& observed_relative_pose) {
  return new ceres::AutoDiffCostFunction<SpaCostFunction2D, 3 /* residuals */,
                                         3 /* start pose variables */,
                                         3 /* end pose variables */>(
      new SpaCostFunction2D(observed_relative_pose));
}

} // anonymous namespace

void ReadPbStream(const std::string& file_name,
                  std::vector<std::array<double, 3>>& pose,
                  std::vector<transform::Rigid3d>& tf);
void OptimizePose(std::vector<std::array<double, 3>>& pose,
                  const std::vector<transform::Rigid3d>& tf);
void WritePbStream(const std::string& read_file,
                   const std::string& write_file,
                   const std::vector<std::array<double, 3>>& pose);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_modify");

    const std::string filename = "state.pbstream";
    const std::string filename2 = "state2.pbstream";

    std::vector<std::array<double, 3>> pose;
    std::vector<transform::Rigid3d> tf;

    // 从原始文件中读取位姿信息，保存在pose和tf中
    ReadPbStream(filename, pose, tf);

    // 以下数据是我们手动标定的
    pose.back()[0] = 1.43;
    pose.back()[1] = 0.0;
    pose.back()[2] = -0.01;

    // 使用Ceres求解优化问题
    OptimizePose(pose, tf);

    WritePbStream(filename, filename2, pose);

    return 0;
}

void ReadPbStream(const std::string& file_name,
                  std::vector<std::array<double, 3>>& pose,
                  std::vector<transform::Rigid3d>& tf)
{
    io::ProtoStreamReader stream(file_name);
    mapping::proto::SerializedData proto;
    mapping::proto::SerializationHeader header;
    stream.ReadProto(&header);

    while(stream.ReadProto(&proto))
    {
        if(proto.has_submap() && proto.submap().has_submap_3d())
        {
            ROS_INFO("Reading submap id %d.",
                     proto.submap().submap_id().submap_index());
            const auto& local_pose = proto.submap().submap_3d().local_pose();
            const transform::Rigid3d cur_tf
                    = {{local_pose.translation().x(),
                        local_pose.translation().y(),
                        local_pose.translation().z()},
                       Eigen::Quaterniond(
                         local_pose.rotation().w(),
                         local_pose.rotation().x(),
                         local_pose.rotation().y(),
                         local_pose.rotation().z())};

            pose.push_back({cur_tf.translation().x(),
                            cur_tf.translation().y(),
                            transform::GetYaw(cur_tf.rotation())});
            tf.push_back(cur_tf);
        }
    }
}

void OptimizePose(std::vector<std::array<double, 3>>& pose,
                  const std::vector<transform::Rigid3d>& tf)
{
    ceres::Problem optimize_problem;
    double trans_w = 100.;
    double rot_w = 1.;

    for(int i = 1; i < pose.size(); i++)
    {
        PoseConstraint rel_pose; // 相对前一个坐标系的位姿
        rel_pose.zbar_ij = tf[i - 1].inverse() * tf[i];
        rel_pose.rotation_weight = rot_w;
        rel_pose.translation_weight = trans_w;

        // 把pose[i]的数据加入优化
        // pose[i]会被更新为优化后的结果。
        optimize_problem.AddResidualBlock(
                    CreateAutoDiffSpaCostFunction(rel_pose),
                    nullptr,
                    pose[i - 1].data(), pose[i].data());
    }

    // 将第一个和最后一个坐标系的位置固定
    optimize_problem.SetParameterBlockConstant(pose.front().data());
    optimize_problem.SetParameterBlockConstant(pose.back().data());

    ceres::Solver::Summary sum;
    ceres::Solver::Options options;
    options.logging_type = ceres::LoggingType::PER_MINIMIZER_ITERATION;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 1000;
    ceres::Solve(options, &optimize_problem, &sum); // 求解优化问题
}

void WritePbStream(const std::string& read_file,
                   const std::string& write_file,
                   const std::vector<std::array<double, 3>>& pose)
{
    io::ProtoStreamReader reader(read_file);
    io::ProtoStreamWriter writer(write_file);
    mapping::proto::SerializedData proto;
    mapping::proto::SerializationHeader header;
    reader.ReadProto(&header);
    writer.WriteProto(header);

    int submap_cnt = 0;

    while(reader.ReadProto(&proto))
    {
        if(proto.has_submap() && proto.submap().has_submap_3d())
        {
            ROS_INFO("processing submap %d", submap_cnt);

            auto local_pose = proto.mutable_submap()->mutable_submap_3d()
                    ->mutable_local_pose();

            local_pose->mutable_translation()->set_x(pose[submap_cnt][0]);
            local_pose->mutable_translation()->set_y(pose[submap_cnt][1]);
            local_pose->mutable_translation()->set_z(0);
            local_pose->mutable_rotation()->set_w(cos(pose[submap_cnt][2] / 2));
            local_pose->mutable_rotation()->set_z(sin(pose[submap_cnt][2] / 2));
            local_pose->mutable_rotation()->set_y(0);
            local_pose->mutable_rotation()->set_x(0);

            submap_cnt++;
        }

        writer.WriteProto(proto);
    }

    writer.Close();
}
