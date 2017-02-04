

import json
import numpy as np
import os

thresholds = np.arange(-2.0, 20.0, 0.4)
print("thresholds", thresholds)

with open('config/pets_initial.json') as f:
    config = json.load(f)

for i, th in enumerate(thresholds):
    # need to change the hit_threshold and output file
    print("Running for threshold th:", th)
    config["detector_opts"]["hit_threshold"] = th
    
    config["output"]["out_filename"] = "out_new/pets/calib_initial/pets_{0:02d}_calib_initial_{1:2f}.txt".format(i, th)
    
    print('Going to save at', config["output"]["out_filename"])

    # save this json in a tmp file
    with open('tmp_config_pets_initial.json', 'w') as out_f:
        json.dump(config, out_f)

    # now runs the detector
    os.system('bin/pedestrian_detection_calibration tmp_config_pets_initial.json')



