from scene.colmap_loader import read_intrinsics_binary,read_extrinsics_text,read_intrinsics_text

cam_extrinsics = read_intrinsics_text('../Downloads/360_v2/cp/sparse/0/cameras.txt')
xys = cam_extrinsics[1]
print(xys)
