/**
* 本程序介绍对Cartographer地图的基本读取、修改操作。
* Cartographer地图是以Protobuf格式保存的，这是Google自己定义的文件存储方式。
* 
**/

// Cartographer自带的读写protobuf的头文件
#include <cartographer/io/proto_stream_deserializer.h>
// Cartographer对于地图的定义
#include <cartographer/io/proto_stream.h>
#include <cartographer/mapping/pose_graph_interface.h>

using namespace cartographer;

void ReWritePbStream(const std::string& read_file,
                     const std::string& write_file);

int main(int argc, char** argv)
{
    const std::string filename = "state.pbstream";
    const std::string filename2 = "state2.pbstream";

    ReWritePbStream(filename, filename2);

    return 0;
}

void ReWritePbStream(const std::string& read_file,
                     const std::string& write_file)
{
    io::ProtoStreamReader reader(read_file);
    io::ProtoStreamWriter writer(write_file);
    mapping::proto::SerializedData proto;
    mapping::proto::SerializationHeader header;
    reader.ReadProto(&header);
    writer.WriteProto(header); // 写文件头header

    while(reader.ReadProto(&proto))
    {
        // 每个地图文件都是由多个proto组成的，例如前面的header就是一个。
        // 每个proto包含的内容不同，比方说子图、轨迹、特征点等等。
        // 我们的目标是，对于含有子图的proto，替换其中子图内容，对于其它proto不操作。
        // 因此需要先用has_submap()方法确定proto中含有子图，然后才能对其操作。
        if(proto.has_submap() && proto.submap().has_submap_3d())
        {
            std::cout << "processing submap "
                      << proto.submap().submap_id().submap_index() << std::endl;

            // 这里我们进行的操作是把高分辨率的地图替换为低分辨率的地图。
            // 注意，如果要修改proto中的一个属性（或者获取可写的引用），要使用带有mutable_前缀的方法
            proto.mutable_submap()->mutable_submap_3d()
                ->mutable_high_resolution_hybrid_grid()->CopyFrom(
                    proto.submap().submap_3d().low_resolution_hybrid_grid());
        }

        writer.WriteProto(proto);
    }

    writer.Close();
}
