from plyfile import PlyData, PlyElement
import numpy as np
from rdp import rdp
import argparse

import py_test

def rdp_fun(basic_path, eps):
    for i in range(0,16):
        file_name = basic_path + '/debug/cut-' + str(i) + '.ply';
        print('read cut cloud file name = ',file_name)
        plydata = PlyData.read(file_name)

        vertex = plydata['vertex']

        (x,y,z) = (vertex[t] for t in ('x', 'y', 'z'))

        x = np.reshape(x, (x.size, 1))
        y = np.reshape(y, (y.size, 1))
        z = np.reshape(z, (z.size, 1))
 
        data = np.concatenate((x,y,z), axis=1)

        rdp_result = rdp(data, epsilon=eps)

        result_file_name = basic_path + '/debug/rdp_result' + str(i) + '.txt';
        np.savetxt(result_file_name, rdp_result, fmt='%.7f', delimiter=' ')
    
        #dbscan.dbscan(rdp_result.t, 0.05, 500)
        print(rdp_result.shape)
        print(rdp_result)


if __name__ == "__main__":
    py_test.fun2()

    parser = argparse.ArgumentParser(description='cut a  curve into several short lines', usage='lidar_cut.py -')
    parser.add_argument('--basic_path', type=str, default = None)
    parser.add_argument('--epsilon', type=float, default = 0.05)
    args = parser.parse_args()

    print('args : basic_path = ' + args.basic_path)
    print('args : epsilon = ' + str(args.epsilon))
    rdp_fun(args.basic_path, args.epsilon)
