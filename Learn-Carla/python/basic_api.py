"""
In this script, we are going to learn how to spawn a vehicle on the road and make it autopilot.
At the same time, we will collect camera and lidar data from it.
"""

import carla
import os
import random
import queue
import time
import collections  

def main():
    actor_list = []
    sensor_list = []
    
    # 创建数据缓冲队列
    image_queue = queue.Queue()
    lidar_queue = queue.Queue()
    
    try:
        # First of all, we need to create the client that will send the requests, assume port is 2000
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        # Retrieve the world that is currently running
        world = client.get_world()
        # world = client.load_world('Town02') # you can also retrive another world by specifically defining
        
        # 启用同步模式
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05  # 20fps
        world.apply_settings(settings)
        
        blueprint_library = world.get_blueprint_library()
        # Set weather for your world
        weather = carla.WeatherParameters(cloudiness=10.0,
                                          precipitation=10.0,
                                          fog_density=10.0)
        world.set_weather(weather)

        # create the ego vehicle
        ego_vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
        # black color
        ego_vehicle_bp.set_attribute('color', '0, 0, 0')
        # get a random valid occupation in the world
        transform = random.choice(world.get_map().get_spawn_points())
        # spawn the vehilce
        ego_vehicle = world.spawn_actor(ego_vehicle_bp, transform)
        # set the vehicle autopilot mode
        ego_vehicle.set_autopilot(True)

        # collect all actors to destroy when we quit the script
        actor_list.append(ego_vehicle)

        # add a camera
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        # 设置相机质量，根据需要可以调整
        camera_bp.set_attribute('image_size_x', '800')
        camera_bp.set_attribute('image_size_y', '600')
        camera_bp.set_attribute('fov', '90')
        
        # 设置相机更新频率
        camera_bp.set_attribute('sensor_tick', '0.1')  # 10Hz
        # camera relative position related to the vehicle
        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=ego_vehicle)

        output_path = '../outputs/output_basic_api'
        if not os.path.exists(output_path):
            os.makedirs(output_path)

        # 在主目录下创建camera和lidar子目录
        camera_output_path = os.path.join(output_path, 'camera')
        lidar_output_path = os.path.join(output_path, 'lidar')
        # 创建帧匹配目录，用于存储同步帧的信息
        paired_output_path = os.path.join(output_path, 'paired')

        if not os.path.exists(camera_output_path):
            os.makedirs(camera_output_path)
        if not os.path.exists(lidar_output_path):
            os.makedirs(lidar_output_path)
        if not os.path.exists(paired_output_path):
            os.makedirs(paired_output_path)

        # 使用同步回调来收集数据
        # 为每种传感器类型使用单独的集合跟踪已处理的帧
        processed_camera_frames = set()
        processed_lidar_frames = set()
        
        # 使用有序字典记录保存的文件，用于检查缺帧
        saved_camera_frames = collections.OrderedDict()
        saved_lidar_frames = collections.OrderedDict()
        
        # 记录帧的对应关系，用于分析数据同步性
        frame_pairs = {}
        
        # 分析帧ID的情况
        camera_frame_ids = []
        lidar_frame_ids = []
        
        # 定义带有tick计数的回调函数
        def camera_callback(image):
            image_queue.put(image)
        
        def lidar_callback(point_cloud):
            lidar_queue.put(point_cloud)
        
        # 设置传感器回调
        camera.listen(camera_callback)
        sensor_list.append(camera)

        # we also add a lidar on it
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', str(32))
        lidar_bp.set_attribute('points_per_second', str(90000))
        lidar_bp.set_attribute('rotation_frequency', str(40))
        lidar_bp.set_attribute('range', str(20))
        # 设置激光雷达更新频率，与相机保持一致
        lidar_bp.set_attribute('sensor_tick', '0.1')  # 10Hz    
        # set the relative location
        lidar_location = carla.Location(0, 0, 2)
        lidar_rotation = carla.Rotation(0, 0, 0)
        lidar_transform = carla.Transform(lidar_location, lidar_rotation)

        # 设置激光雷达回调
        lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=ego_vehicle)
        lidar.listen(lidar_callback)
        sensor_list.append(lidar)

        # 运行时统计信息
        total_camera_frames = 0
        total_lidar_frames = 0
        synchronized_frames = 0  # 同时具有相机和激光雷达数据的帧数
        sim_frames = 0  # 模拟器实际步进的帧数
        last_tick_timestamp = time.time()
        start_time = time.time()
        
        # 启用交通管理器的同步模式
        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)

        # 创建日志文件用于记录帧ID
        log_file = open(os.path.join(output_path, 'frame_log.txt'), 'w')
        log_file.write("时间戳,仿真帧,相机帧ID,激光雷达帧ID\n")
        
        print("开始数据采集，按Ctrl+C停止...")
        
        # 用于记录最近一段时间的帧号，便于分析规律
        recent_camera_frames = collections.deque(maxlen=20)
        recent_lidar_frames = collections.deque(maxlen=40)
        
        # 最近处理的世界帧
        last_world_frame = None
        
        while True:
            # 计算当前帧率
            current_time = time.time()
            if current_time - last_tick_timestamp > 2.0:  # 每2秒打印一次帧率
                fps = sim_frames / (current_time - last_tick_timestamp)
                print(f"当前帧率: {fps:.2f} FPS, 相机帧数: {total_camera_frames}, 激光雷达帧数: {total_lidar_frames}, 同步帧数: {synchronized_frames}")
                
                # 输出最近的帧ID来分析规律
                if recent_camera_frames and recent_lidar_frames:
                    print(f"最近相机帧ID: {list(recent_camera_frames)[-5:]} (数量: {len(recent_camera_frames)})")
                    print(f"最近激光雷达帧ID: {list(recent_lidar_frames)[-5:]} (数量: {len(recent_lidar_frames)})")
                
                sim_frames = 0
                last_tick_timestamp = current_time
            
            # 步进仿真
            world_frame = world.tick()
            sim_frames += 1
            last_world_frame = world_frame
            
            # 更新观察视角
            try:
                if ego_vehicle.is_alive:
                    spectator = world.get_spectator()
                    transform = ego_vehicle.get_transform()
                    spectator.set_transform(carla.Transform(transform.location + carla.Location(z=20),
                                                          carla.Rotation(pitch=-90)))
                else:
                    print("车辆不再存在，退出循环")
                    break
            except RuntimeError:
                print("车辆已销毁，退出循环")
                break
                
            # 处理相机队列
            try:
                while not image_queue.empty():
                    image = image_queue.get()
                    
                    # 如果此帧已处理过，则跳过
                    if image.frame in processed_camera_frames:
                        continue
                    
                    # 记录帧ID用于分析
                    camera_frame_ids.append(image.frame)
                    recent_camera_frames.append(image.frame)
                    
                    # 记录到日志
                    log_file.write(f"{current_time:.3f},{world_frame},{image.frame},\n")
                    
                    # 标记为已处理
                    processed_camera_frames.add(image.frame)
                    
                    # 保存图像
                    filename = f'{image.frame:06d}.png'
                    image.save_to_disk(os.path.join(camera_output_path, filename))
                    
                    # 记录已保存的帧
                    saved_camera_frames[image.frame] = filename
                    total_camera_frames += 1
                    
                    # 查找匹配的激光雷达帧
                    # 探索几种可能的匹配策略
                    matched_lidar_frames = []
                    
                    # 1. 直接相同帧号
                    if image.frame in processed_lidar_frames:
                        matched_lidar_frames.append(image.frame)
                    
                    # 2. 相机帧ID附近的激光雷达帧
                    for offset in [-1, 0, 1]:
                        potential_frame = image.frame + offset
                        if potential_frame in processed_lidar_frames and potential_frame != image.frame:
                            matched_lidar_frames.append(potential_frame)
                    
                    # 如果找到匹配帧
                    if matched_lidar_frames:
                        for lidar_frame in matched_lidar_frames:
                            if lidar_frame not in frame_pairs:
                                synchronized_frames += 1
                                frame_pairs[lidar_frame] = image.frame
                                
                                # 可以将匹配的帧号记录到文件中
                                with open(os.path.join(paired_output_path, f'{image.frame:06d}_{lidar_frame:06d}.txt'), 'w') as f:
                                    f.write(f"相机帧: {image.frame}, 激光雷达帧: {lidar_frame}\n")
                    
                    # 保持字典大小合理，避免内存泄漏
                    if len(saved_camera_frames) > 1000:
                        # 删除最老的500个条目
                        for _ in range(500):
                            if saved_camera_frames:
                                saved_camera_frames.popitem(last=False)
            except Exception as e:
                print(f"处理相机数据时出错: {e}")
            
            # 处理激光雷达队列
            try:
                while not lidar_queue.empty():
                    point_cloud = lidar_queue.get()
                    
                    # 如果此帧已处理过，则跳过
                    if point_cloud.frame in processed_lidar_frames:
                        continue
                    
                    # 记录帧ID用于分析
                    lidar_frame_ids.append(point_cloud.frame)
                    recent_lidar_frames.append(point_cloud.frame)
                    
                    # 记录到日志
                    log_file.write(f"{current_time:.3f},{world_frame},,{point_cloud.frame}\n")
                    
                    # 标记为已处理
                    processed_lidar_frames.add(point_cloud.frame)
                    
                    # 保存点云数据
                    filename = f'{point_cloud.frame:06d}.ply'
                    point_cloud.save_to_disk(os.path.join(lidar_output_path, filename))
                    
                    # 记录已保存的帧
                    saved_lidar_frames[point_cloud.frame] = filename
                    total_lidar_frames += 1
            except Exception as e:
                print(f"处理激光雷达数据时出错: {e}")
            
            # 适当休眠，避免CPU使用率过高
            time.sleep(0.001)
            
            # 限制集合大小，避免内存泄漏
            if len(processed_camera_frames) > 1000:
                processed_camera_frames = set(list(processed_camera_frames)[-500:])
            if len(processed_lidar_frames) > 1000:
                processed_lidar_frames = set(list(processed_lidar_frames)[-500:])
            if len(frame_pairs) > 1000:
                frame_pairs = {k: frame_pairs[k] for k in list(frame_pairs.keys())[-500:]}

    finally:
        # 关闭日志文件
        if 'log_file' in locals() and not log_file.closed:
            log_file.close()
            
        # 计算总运行时间
        total_time = time.time() - start_time
        
        print('destroying actors')
        # 恢复异步模式
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)
        
        # 先停止所有传感器监听
        for sensor in sensor_list:
            sensor.stop()
            
        # 等待队列处理完毕
        time.sleep(0.5)
        
        # 销毁传感器
        for sensor in sensor_list:
            sensor.destroy()
        
        # 销毁车辆
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        
        # 分析保存的帧并检测缺失
        print("\n--- 数据采集统计 ---")
        
        # 相机数据分析
        if saved_camera_frames:
            camera_frames = sorted(list(saved_camera_frames.keys()))
            first_camera_frame = camera_frames[0]
            last_camera_frame = camera_frames[-1]
            expected_camera_frames = last_camera_frame - first_camera_frame + 1
            actual_camera_frames = len(camera_frames)
            
            print(f"相机数据: {actual_camera_frames} 帧")
            if expected_camera_frames != actual_camera_frames:
                missing = expected_camera_frames - actual_camera_frames
                missing_percent = (missing / expected_camera_frames) * 100 if expected_camera_frames > 0 else 0
                print(f"  - 检测到缺失: 期望 {expected_camera_frames} 帧, 丢失 {missing} 帧 ({missing_percent:.2f}%)")
                
            # 检查是否只有奇数帧或偶数帧
            odd_frames = sum(1 for f in camera_frames if f % 2 == 1)
            even_frames = sum(1 for f in camera_frames if f % 2 == 0)
            print(f"  - 奇数帧: {odd_frames}, 偶数帧: {even_frames}")
            if odd_frames > 0 and even_frames == 0:
                print("  - 相机只使用奇数帧")
            elif odd_frames == 0 and even_frames > 0:
                print("  - 相机只使用偶数帧")
        
        # 激光雷达数据分析
        if saved_lidar_frames:
            lidar_frames = sorted(list(saved_lidar_frames.keys()))
            first_lidar_frame = lidar_frames[0]
            last_lidar_frame = lidar_frames[-1]
            expected_lidar_frames = last_lidar_frame - first_lidar_frame + 1
            actual_lidar_frames = len(lidar_frames)
            
            print(f"激光雷达数据: {actual_lidar_frames} 帧")
            if expected_lidar_frames != actual_lidar_frames:
                missing = expected_lidar_frames - actual_lidar_frames
                missing_percent = (missing / expected_lidar_frames) * 100 if expected_lidar_frames > 0 else 0
                print(f"  - 检测到缺失: 期望 {expected_lidar_frames} 帧, 丢失 {missing} 帧 ({missing_percent:.2f}%)")
                
            # 检查是否只有奇数帧或偶数帧
            odd_frames = sum(1 for f in lidar_frames if f % 2 == 1)
            even_frames = sum(1 for f in lidar_frames if f % 2 == 0)
            print(f"  - 奇数帧: {odd_frames}, 偶数帧: {even_frames}")
            if odd_frames > 0 and even_frames == 0:
                print("  - 激光雷达只使用奇数帧")
            elif odd_frames == 0 and even_frames > 0:
                print("  - 激光雷达只使用偶数帧")
        
        # 帧ID分析
        if camera_frame_ids and lidar_frame_ids:
            print("\n帧ID模式分析:")
            if len(camera_frame_ids) >= 3:
                camera_diffs = [camera_frame_ids[i+1] - camera_frame_ids[i] for i in range(len(camera_frame_ids)-1)]
                print(f"相机帧ID差值: {camera_diffs[:5]}...")
                common_diff = collections.Counter(camera_diffs).most_common(1)[0]
                print(f"最常见的相机帧差值: {common_diff[0]} (出现 {common_diff[1]} 次)")
            
            if len(lidar_frame_ids) >= 3:
                lidar_diffs = [lidar_frame_ids[i+1] - lidar_frame_ids[i] for i in range(len(lidar_frame_ids)-1)]
                print(f"激光雷达帧ID差值: {lidar_diffs[:5]}...")
                common_diff = collections.Counter(lidar_diffs).most_common(1)[0]
                print(f"最常见的激光雷达帧差值: {common_diff[0]} (出现 {common_diff[1]} 次)")
        
        # 同步率分析
        if total_camera_frames > 0 and total_lidar_frames > 0:
            sync_rate = (synchronized_frames / max(total_camera_frames, total_lidar_frames)) * 100
            print(f"\n传感器同步率: {sync_rate:.2f}% ({synchronized_frames} 个同步帧)")
            
            if len(frame_pairs) > 0:
                print("帧匹配样例:")
                sample_count = min(5, len(frame_pairs))
                samples = list(frame_pairs.items())[:sample_count]
                for lidar_frame, camera_frame in samples:
                    print(f"  激光雷达帧 {lidar_frame} 匹配到相机帧 {camera_frame}")
        
        # 计算平均帧率
        avg_camera_fps = total_camera_frames / total_time if total_time > 0 else 0
        avg_lidar_fps = total_lidar_frames / total_time if total_time > 0 else 0
        
        print(f'\n总运行时间: {total_time:.2f}秒')
        print(f'平均相机帧率: {avg_camera_fps:.2f} FPS')
        print(f'平均激光雷达帧率: {avg_lidar_fps:.2f} FPS')
        print(f'总帧数 - 相机: {total_camera_frames}, 激光雷达: {total_lidar_frames}, 同步: {synchronized_frames}')
        print('done.')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')

