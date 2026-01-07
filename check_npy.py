import numpy as np

# 파일 이름
filename = 'joint_data.npy'

try:
    # .npy 파일 불러오기
    data = np.load(filename)

    print(f"===== Successfully loaded '{filename}' =====")

    # 데이터의 모양(shape) 출력
    # (행의 수, 열의 수) -> (데이터 포인트 수, 관절 수)
    print(f"Data shape: {data.shape}")

    print("\n--- First 5 rows of data: ---")
    # 데이터의 첫 5줄만 출력해서 내용 확인
    print(data[:5])

    print("\n--- Last 5 rows of data: ---")
    # 데이터의 마지막 5줄만 출력해서 내용 확인
    print(data[-5:])

except FileNotFoundError:
    print(f"Error: The file '{filename}' was not found in this directory.")
except Exception as e:
    print(f"An error occurred: {e}")
