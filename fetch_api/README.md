# fetch_api

A Python wrapper for Fetch's hardware. [Originally developed](https://github.com/cse481wi18/cse481wi18/tree/indigo-devel/fetch_api) by Justin Huang for use in UW's undergraduate robotics capstone course, CSE481.

## Usage

The Arm, Base, Gripper and Head classes wrap the robot's default controllers. This reduces the boilerplate that comes with setting up action and service clients associated with these controllers.

To explore the API, try out the `ifetch` REPL.

    `rosrun fetch_api ifetch`

You'll find an instance of each of the main classes available, and you can see their methods in the tab completion. iPython also exposes each method's documentation via `?`. To see the documentation for `head.look_at`:

    head.look_at?
