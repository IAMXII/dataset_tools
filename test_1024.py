import open3d as o3d
import numpy as np
import cv2

fx = 332.232689  # x 轴方向的焦距
fy = 332.644823  # y 轴方向的焦距
cx = 333.058485  # x 轴方向的光心坐标
cy = 240.998586  # y 轴方向的光心坐标
big_num = 1e10
# 构建相机内参矩阵
K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0, 0, 1]])
# P = np.array([[-0.0264814146660713, -0.999649386885150, 0.00173906523035761, 0.0659530032036776],
#               [0.0121395165905959, -0.00206233611926795, -0.999925722855674, 0.108374858163385],
#               [0.999576628994339, -0.0264570578759927, 0.0121885681396033, 0.0699178702997177]])
P = np.array([[-0.01783068, -0.99978912, 0.01020197, -0.08032920],
              [-0.02988329, -0.00966631, -0.99950683, 0.10752965],
              [0.99939454, -0.01812658, -0.02970480, -0.06024161]])


# 步骤 1: 构建八叉树
def build_octree(point_cloud_file, max_depth=4):
    # 加载点云数据
    pcd = o3d.io.read_point_cloud(point_cloud_file)
    #
    # 构建八叉树
    octree = o3d.geometry.Octree(max_depth=max_depth)
    octree.convert_from_point_cloud(pcd, size_expand=0.01)
    o3d.visualization.draw_geometries([octree])
    return octree


# 步骤 2: 获取特定深度的八叉树节点边界
def get_octree_bboxes_at_depth(octree, target_depth):
    bboxes = []

    def extract_node(node, depth):
        # print(depth.depth)
        # print(depth.depth == target_depth)
        # print(node)
        if depth.depth == target_depth and isinstance(node, o3d.geometry.OctreeLeafNode):
            # print(depth.depth)
            # 获取叶子节点的包围盒 (bounding box)
            # print(octree.size)
            bbox_min = depth.origin
            bbox_size = octree.size / (2 ** depth.depth)
            bbox_max = bbox_min + bbox_size
            bboxes.append((bbox_min, bbox_max))
        return False

    octree.traverse(extract_node)
    # print(bboxes)
    return bboxes


# 步骤 3: 将八叉树节点边界转换为相机坐标系
def transform_to_camera_coords(points, R, t):
    return np.dot(R, points.T).T + t


# 步骤 4: 将3D点投影到图像平面
def project_to_image(points_3d, K):
    projected_points = np.dot(K, points_3d.T).T
    projected_points[:, 0] /= projected_points[:, 2]
    projected_points[:, 1] /= projected_points[:, 2]
    return projected_points[:, :2]


# 步骤 5: 绘制投影结果到图像上
def draw_projected_bboxes(mask, projected_bboxes):
    for projected_bbox in projected_bboxes:
        print("step:",projected_bbox)
        polygon = np.array(projected_bbox, dtype=np.int32)
        # cv2.polylines(image, [polygon], isClosed=True, color=(0, 255, 0), thickness=1)
        cv2.fillPoly(mask, [polygon], 0)
    return mask

def draw_projected_bboxes_box(image, projected_bboxes):
    for projected_bbox in projected_bboxes:

        print("step:",projected_bbox)
        polygon = np.array(projected_bbox, dtype=np.int32)
        cv2.polylines(image, [polygon], isClosed=True, color=(0, 255, 0), thickness=1)
        # cv2.fillPoly(mask, [polygon], 0)
    return image
# 主函数
def main():
    # 输入文件
    point_cloud_file = "../dataset/cp/pcds_lidar/pointcloud_0.pcd"  # 替换为你的点云文件
    image_file = "../dataset/cp/images_lidar_2/image_000000.jpg"  # 替换为你的图像文件
    # output_image_file = "image.png"

    # 加载图像

    # 构建八叉树，设定最大深度为4
    for max_depth in range(10, 11):

        image = cv2.imread(image_file)
        mask = np.ones_like(image)

        output_image_file = "image_{}.png".format(max_depth)
        octree = build_octree(point_cloud_file, max_depth=max_depth)

        # 获取第4级（depth=4）的八叉树节点边界
        bboxes = get_octree_bboxes_at_depth(octree, target_depth=max_depth)
        # print(bboxes)
        # 相机的旋转矩阵（假设）
        R = P[:3, :3]

        # 相机的平移向量（假设）
        t = P[:3, 3]

        # 相机内参矩阵（假设）

        # 将八叉树边界转换到相机坐标系并投影到图像平面
        bboxes_camera = []

        for bbox_min, bbox_max in bboxes:
            bbox_1 = bbox_min
            bbox_2 = [bbox_min[0], bbox_max[1], bbox_min[2]]
            bbox_3 = [bbox_min[0], bbox_max[1], bbox_max[2]]
            bbox_4 = [bbox_min[0], bbox_min[1], bbox_max[2]]
            bbox_5 = [bbox_max[0], bbox_min[1], bbox_min[2]]
            bbox_6 = [bbox_max[0], bbox_max[1], bbox_min[2]]
            bbox_7 = [bbox_max[0], bbox_max[1], bbox_max[2]]
            bbox_8 = [bbox_max[0], bbox_min[1], bbox_max[2]]
            plane_1 = np.array([bbox_1, bbox_2, bbox_3, bbox_4])
            plane_2 = np.array([bbox_5, bbox_6, bbox_7, bbox_8])
            plane_3 = np.array([bbox_2, bbox_3, bbox_7, bbox_6])
            plane_4 = np.array([bbox_1, bbox_4, bbox_8, bbox_5])
            plane_5 = np.array([bbox_3, bbox_4, bbox_8, bbox_7])
            plane_6 = np.array([bbox_1, bbox_2, bbox_6, bbox_5])
            # corners2 = np.array([[bbox_max[0], bbox_max[1], bbox_min[2]],
            #                      bbox_max,
            #                      [bbox_max[0], bbox_min[1], bbox_max[2]],
            #
            #                      [bbox_max[0], bbox_min[1], bbox_min[2]]])
            # 转换到相机坐标
            plane = [plane_1,plane_2]#, plane_3, plane_4,plane_5,plane_6
            for plane_bbox in plane:

                corners_camera = transform_to_camera_coords(plane_bbox, R, t)
                bboxes_camera.append(corners_camera)
            # print(corners_camera)

            # bboxes_camera.append(corners_camera2)

        # 投影八叉树节点到图像平面
        projected_bboxes = []
        for corners_camera in bboxes_camera:
            projected_bbox = project_to_image(corners_camera, K)
            # print(projected_bbox[0])
            projected_bboxes.append(projected_bbox)

        # 绘制八叉树投影边界到图像上
        # output_image = draw_projected_bboxes_box(image, projected_bboxes)
        mask = draw_projected_bboxes(mask, projected_bboxes)
        output_image = image*(1-mask)
        # 保存结果图像
        cv2.imwrite(output_image_file, output_image)
        # cv2.imshow('Projected Octree', output_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
