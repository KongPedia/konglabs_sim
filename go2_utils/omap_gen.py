import os
import yaml
import numpy as np
from PIL import Image

# Isaac Sim 및 Carb 임포트
import carb
from isaacsim.asset.gen.omap.bindings import _omap
import omni.physx
import omni.usd

def generate_nav2_map(cfg):
    print("[INFO] Generating Occupancy Map using Generator API...")
    
    # 1. 파일 경로 설정
    output_path = getattr(cfg.mapping, "output_path", os.getcwd())
    map_name = "nav2_map"
    
    pgm_filename = f"{map_name}.png"
    yaml_filename = f"{map_name}.yaml"
    full_pgm_path = os.path.join(output_path, pgm_filename)
    full_yaml_path = os.path.join(output_path, yaml_filename)

    os.makedirs(output_path, exist_ok=True)

    # 2. 인터페이스 획득
    physx_interface = omni.physx.get_physx_interface()
    stage_id = omni.usd.get_context().get_stage_id()

    # 3. Generator 생성
    generator = _omap.Generator(physx_interface, stage_id)
    
    # [설정] Nav2 맵 범위 및 스캔 높이 수정
    min_bound = cfg.mapping.lower_bound
    max_bound = cfg.mapping.upper_bound
    origin = cfg.mapping.origin # 스캔 시작점도 안전하게 띄웁니다.
    
    # 문서 참조: update_settings(cell_size, occupied_val, unoccupied_val, unknown_val)
    # 내부 계산용 값 설정 (이미지 생성 시에는 색상으로 덮어씌워지지만 설정은 필요)
    cell_size = cfg.mapping.cell_size
    generator.update_settings(cell_size, 4, 5, 6)
    
    # 문서 참조: set_transform(origin, min_bound, max_bound)
    generator.set_transform(origin, min_bound, max_bound)

    # 4. 맵 데이터 계산 (2D Flattened)
    print("[INFO] Computing 2D occupancy...")
    generator.generate2d()

    # 5. [문서 반영] get_colored_byte_buffer 사용
    # 문서 파라미터: (Occupied Color, Unoccupied Color, Unknown Color)
    # Nav2 표준 색상 (Occupied=Black, Free=White, Unknown=Gray)
    # 타입: carb.Int4 (R, G, B, A)
    
    occupied_color = carb.Int4(0, 0, 0, 255)
    free_color = carb.Int4(255, 255, 255, 255)
    unknown_color = carb.Int4(127, 127, 127, 255)
    
    buffer_list = generator.get_colored_byte_buffer(occupied_color, free_color, unknown_color)

    # 6. 이미지 변환 및 저장
    dims = generator.get_dimensions() 
    width = dims.x
    height = dims.y
    
    if not buffer_list or len(buffer_list) == 0:
        print("[ERROR] Generated buffer is empty.")
        return

    # [수정] 문자(char) 리스트를 정수(int) 리스트로 변환
    # 만약 buffer_list가 이미 숫자라면 이 코드는 그대로 동작하거나, 
    # 에러가 난다면 타입을 확인해서 분기 처리해야 합니다.
    # 현재 에러 상황('ÿ')을 해결하기 위해 ord()를 사용하거나 map을 씁니다.

    try:
        # Case A: 리스트 요소가 정수인 경우 (정상)
        img_array = np.array(buffer_list, dtype=np.uint8)
    except ValueError:
        # Case B: 리스트 요소가 문자('ÿ')인 경우 (현재 에러 상황)
        # 문자 하나하나를 ord()로 숫자로 바꿉니다.
        # 주의: buffer_list가 엄청 길 수 있으므로 리스트 컴프리헨션보다 map이 빠를 수 있음
        img_array = np.array([ord(c) for c in buffer_list], dtype=np.uint8)

    img_array = img_array.reshape((height, width, 4))
    
    im = Image.fromarray(img_array, mode="RGBA")
    # Nav2 호환을 위해 Grayscale 변환
    im = im.convert("L")
    

    im = im.transpose(Image.FLIP_LEFT_RIGHT) # 좌우 반전
    im = im.transpose(Image.FLIP_TOP_BOTTOM) # 상하 반전 (Nav2 원점 맞춤) [1]
    im = im.convert("L")
    im.save(full_pgm_path)
    print(f"[INFO] Map image saved to: {full_pgm_path}")

    # 7. YAML 파일 생성
    nav2_yaml = {
        "image": pgm_filename,
        "mode": "trinary",
        "resolution": cell_size,
        "origin": [origin[0] + min_bound[0], origin[1] + min_bound[1], 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25
    }

    with open(full_yaml_path, 'w') as f:
        yaml.dump(nav2_yaml, f, sort_keys=False)
    
    print(f"[INFO] Nav2 YAML saved to: {full_yaml_path}")