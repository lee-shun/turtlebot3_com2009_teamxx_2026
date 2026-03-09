# AGENTS.md

## Build, Lint, and Test Commands

### Build Commands
```bash
# Build the entire workspace
colcon build

# Build specific package
colcon build --packages-select com2009_teamxx_2026

# Build with verbose output
colcon build --symlink-install --packages-select com2009_teamxx_2026 -v
```

### Source Commands
```bash
# Source the workspace
source install/setup.bash

# Source in current shell
. install/setup.bash
```

### Lint Commands
```bash
# Run linters (ament_lint_auto)
ament_lint_auto --select=all --packages-select com2009_teamxx_2026

# Run specific linters
ament_lint_copyright --packages-select com2009_teamxx_2026
ament_flake8 --packages-select com2009_teamxx_2026
ament_cpplint --packages-select com2009_teamxx_2026
```

### Test Commands
```bash
# Run all tests in the package
colcon test --packages-select com2009_teamxx_2026

# Run specific test file
colcon test --packages-select com2009_teamxx_2026 --event-handlers console_direct+

# View test results
colcon test-result --all --verbose

# Run tests with pytest directly (if tests are structured that way)
pytest scripts/task1_test.py -v
pytest scripts/task1_test2.py -v
```

### Development Workflow
```bash
# Full development cycle
colcon build --symlink-install
source install/setup.bash
colcon test
```

## Code Style Guidelines

### Python Code Style

#### Imports
- Import all modules at the top of the file
- Use absolute imports for ROS packages (e.g., `from rclpy.node import Node`)
- Group imports: ROS imports, local module imports, standard library imports
- Example:
  ```python
  import rclpy
  from rclpy.node import Node
  from geometry_msgs.msg import TwistStamped
  from nav_msgs.msg import Odometry
  import math
  import time
  ```

#### Type Hints
- Use type hints for function parameters and return values
- Follow ROS message type conventions (e.g., `msg:Odometry`, `lin_x: float`)
- Example:
  ```python
  def odom_callback(self, msg: Odometry):
  def publish_cmd(self, lin_x: float, ang_z: float):
  ```

#### Class Structure
- Inherit from `rclpy.node.Node` for ROS nodes
- Use descriptive class names (PascalCase)
- Initialize publishers and subscribers in `__init__`
- Use super().__init__() to call parent constructor
- Example:
  ```python
  class FigOfEightNode(Node):
      def __init__(self):
          super().__init__('fig_of_right_node')
          self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
  ```

#### Naming Conventions
- **Classes**: PascalCase (e.g., `FigOfEightNode`, `Circle`)
- **Functions**: snake_case (e.g., `odom_callback`, `control_step`, `print_odom_log`)
- **Private methods**: prefix with underscore (e.g., `_shutdown_done`)
- **Variables**: snake_case (e.g., `initial_x`, `linear_vel`, `angular_vel`)
- **Constants**: UPPER_SNAKE_CASE (not strictly enforced but recommended)
- **ROS topics**: snake_case with forward slashes (e.g., `'/cmd_vel'`, `'odom'`)

#### Code Formatting
- Use 4 spaces for indentation
- Line length: aim for 80-120 characters (follow PEP 8)
- Add blank lines between logical sections
- Use descriptive variable names
- Avoid single-letter variable names except for loop counters

#### Error Handling
- Use try-except blocks for ROS operations
- Handle `KeyboardInterrupt` gracefully
- Use `rclpy.shutdown()` in finally blocks
- Ensure cleanup in shutdown handlers
- Example:
  ```python
  try:
      rclpy.spin(fig8)
  except KeyboardInterrupt:
      print(f"{fig8.get_name()} received a shutdown request (Ctrl+C)")
  finally:
      fig8.on_shutdown()
      fig8.destroy_node()
      rclpy.shutdown()
  ```

#### Comments and Documentation
- Add docstrings for classes and methods (especially public ones)
- Use inline comments for complex logic
- Comment important variables and their units
- Example:
  ```python
  #-- odom calibration
  self.odom_calibrated = False
  self.initial_x = 0.0

  def odom_callback(self, msg: Odometry):
      """Callback to process odometry messages."""
      q = (
          msg.pose.pose.orientation.x,
          msg.pose.pose.orientation.y,
          msg.pose.pose.orientation.z,
          msg.pose.pose.orientation.w,
      )
  ```

#### Logging
- Use `self.get_logger()` for node-specific logging
- Use `get_logger().info()` for general information
- Use `get_logger().warn()` for warnings
- Use `get_logger().error()` for errors
- Include units in log messages (e.g., `[m]`, `[rad/s]`, `[degrees]`)
- Example:
  ```python
  self.get_logger().info(f"x={self.current_x_rel:.2f} [m], y={self.current_y_rel:.2f} [m]")
  ```

#### ROS-Specific Patterns
- Initialize ROS with `rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)`
- Use `create_publisher()` for command publishing
- Use `create_subscription()` for data receiving
- Use `create_timer()` for periodic callbacks (e.g., 0.1s for control loops)
- Use `create_rate()` for rate-limited operations
- Handle ROS node lifecycle: init → spin → shutdown

#### Math and Physics
- Use `math` module for mathematical operations
- Convert between degrees and radians using `math.radians()` and `math.degrees()`
- Use `time` module for timing operations
- Handle floating-point comparisons carefully (use tolerance for position checks)

## Project Structure

```
com2009_teamxx_2026/
├── com2009_teamxx_2026_modules/     # Python package modules
│   ├── __init__.py
│   └── tb3_tools.py                 # Utility functions (e.g., quaternion conversion)
├── scripts/                         # Executable Python scripts
│   ├── basic_velocity_control.py
│   ├── stop_me.py
│   ├── task1.py
│   ├── task1_test.py
│   └── task1_test2.py
├── launch/                          # ROS 2 launch files
│   └── task1.launch.py
├── CMakeLists.txt                   # Build configuration
├── package.xml                      # Package metadata
└── AGENTS.md                        # This file
```

## Best Practices

1. **Testing**: Always run tests before committing changes
2. **Single Responsibility**: Each node should have a clear purpose
3. **Modularity**: Use the `tb3_tools.py` module for shared utilities
4. **Safety**: Always publish zero velocity when stopping
5. **State Management**: Track robot state clearly (e.g., `self.state`)
6. **Time Management**: Use ROS clock for timing rather than system time
7. **Resource Management**: Clean up publishers, subscribers, and timers
8. **Logging**: Provide informative logs for debugging and monitoring
9. **Type Safety**: Use type hints for better code clarity and debugging
10. **Documentation**: Add comments explaining complex algorithms
