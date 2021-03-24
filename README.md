# RosPackageExample

This is an example package for ROS working with catkin build system.

## Creating a new repository for your existing ROS package

1. Create via GitHub (just clicking) a new repository without readme.md, .gitignore or license
2. On your computer, under catkin_ws/src/your_package_name, open a prompt and initialize your git repository with : 
 * git init
3. Add existing files to the repository : 
 * git add .
4. Commit this repository for the first time : 
 * git commit -m "First commit"
5. On GitHub, get your remote repository url, it looks like https://github.com/ISENRobotics/your_package_name.git and copy it to clipboard
6. Return to your prompt and add the remote url to your repository : 
 * git remote add origin https://github.com/ISENRobotics/your_package_name.git
7. Puch your local repo to the remote url : 
 * git push -u origin master

## Configuring git on your computer
The folowing commands are needed to contribute to github repositories : 
 * git config --global user.name "YOUR NAME"
 * git config --global user.email "YOUR EMAIL ADDRESS"
