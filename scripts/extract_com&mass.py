import re


parts = [
    {"mass":  0.68375, "centerOfMass": [-0.000992693,0,0.0434047]},
    {"mass":  0.06483, "centerOfMass": [-0.02744,0,-0.00014]},
    {"mass":  0.25081, "centerOfMass": [0.0148309,0.00633711,0.00212777]},
    {"mass":  0.06483, "centerOfMass": [-0.02744,0,-0.00014]},
    {"mass":  0.25081, "centerOfMass": [0.0148309,-0.00633711,0.00212777]},
    {"mass":  0.09127, "centerOfMass": [0.02542,-0.0033,-0.03239]},
    {"mass":  0.13416, "centerOfMass": [0.00045,-0.00029,0.00685]},
    {"mass":  0.30142, "centerOfMass": [0.00453,-0.00225,-0.04936]},
    {"mass":  0.38968, "centerOfMass": [0.00138,-0.00221,-0.05373]},
    {"mass":  0.14053, "centerOfMass": [-0.01549,-0.00029,-0.00515]},
    {"mass":  0.06981, "centerOfMass": [-0.00781,0.01114,0.02661]},
    {"mass":  0.09127, "centerOfMass": [0.02542,0.0033,-0.03239]},
    {"mass":  0.13416, "centerOfMass": [0.00045,0.00029,0.00685]},
    {"mass":  0.30142, "centerOfMass": [0.00453,0.00225,-0.04936]},
    {"mass":  0.38968, "centerOfMass": [0.00138,0.00221,-0.05373]},
    {"mass":  0.14053, "centerOfMass": [-0.01549,0.00029,-0.00515]},
    {"mass":  0.06981, "centerOfMass": [-0.00781,-0.01114,0.02661]},
    {"mass":  1.04956, "centerOfMass": [-0.00413,0,0.04342]},
]


def calculate_overall_com(parts):
    total_mass = sum(part['mass'] for part in parts)
    overall_com = [0.0, 0.0, 0.0]

    for part in parts:
        mass = part['mass']
        com = part['centerOfMass']

        overall_com[0] += mass * com[0]
        overall_com[1] += mass * com[1]
        overall_com[2] += mass * com[2]

    # 计算加权平均质心位置
    overall_com = [coord / total_mass for coord in overall_com]
    return overall_com


# 使用示例
proto_file = '/Users/xuzhihong/Desktop/Nao_test/protos/Nao.proto'  # 将此文件名替换为您的 .proto 文件路径

if parts:
    overall_com = calculate_overall_com(parts)
    print("整体质心位置:", overall_com)
else:
    print("未找到任何部件的 mass 或 centerOfMass 信息")