# Soft_Inflatable_Robot_Arm

### Install Python

### Install anaconda
https://docs.conda.io/projects/conda/en/stable/user-guide/install/index.html

### Create anaconda env
`conda env create -n SixDoF --file 6dof.yml`

`conda activate SixDoF`

### Install and activate DynamixelSDK

This procedure should be done outside of 'Soft_Inflatable_Robot_Arm' repo.

`(SixDoF)> git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git`

`(SixDoF)> cd DynamixelSDK/python`

`(SixDoF)> python setup.py install`

### Install pre-commit
`conda install -c conda-forge pre-commit`

### Github Naming Convention
`git checkout -B feature/<user-name>-<content>`

example

`git checkout -B featur/Masato-debug_data_collection`
