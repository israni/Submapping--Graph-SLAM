**About**

This repository contains the final project for EECS/NAVARCH 568 Winter 2018, Group 13. In this project, we explore the effects of submapping and loop closures on GraphSLAM. 

We use the GTSAM MATLAB toolbox (https://borg.cc.gatech.edu/) to solve the back-end optimization, and TORO-formatted data sets (https://lucacarlone.mit.edu/datasets/) as the factor graph inputs.

**How to Use**

*Set up*

In MATLAB, navigate to the base directory. If running in Windows, right-click 'gtsam_toolbox_windows' and select 'Add to Path > Selected Folders' (NOT 'Selected Folders and Subfolders'). If in Linux, choose the 'gtsam_toolbox_windows'. Mac users can download the Mac version of the MATLAB toolbox from the provided link in the About section. Also add the 'Datasets' folder to the path.

*Using read_data_toro.m -- no submapping*

Uncomment the data_file and corresponding num_points for the desired data set. Running this script will optimize over the whole factor graph without submapping.
Outputs:
- Run time
- Final map

*Using graph_optimize_toro.m -- submapping*

Choose the number of desired submaps (num_submaps), and uncomment the num_points_total and file_name for the desired data set. Running this script will create the specified number of submaps, optimize over the submaps, then create the global optimzed map. 
Outputs:
- Run time
- Submap optimizations
- Final map

/////////////

Group 13 consists of the following members:
- Sagar Israni: israni@umich.edu
- Rachel Vitali: vitalir@umich.edu
- Jihong Wang: jihwang@umich.edu
- Esther Yan: eeyan@umich.edu
- Zihao Zhang: ctsiho@umich.edu
