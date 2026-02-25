<div align="center">
  <h1>🤖 Dexmate Robot Control and Sensing API</h1>
</div>

![Python](https://img.shields.io/badge/python-3.10%20%7C%203.11%20%7C%203.12%20%7C%203.13-blue)

## 📦 Installation

```shell
pip install dexcontrol
```

To run the examples in this repo, you can try:

```shell
pip install dexcontrol[example]
```

### Robotiq 2F-85 Gripper (optional)

If you want to use a Robotiq 2F-85 gripper connected via USB-RS485, clone this
repo with submodules and install the gripper package:

```shell
# Clone with submodule
git clone --recurse-submodules https://github.com/configint/custom_dexcontrol.git

# If you already cloned without --recurse-submodules:
git submodule update --init --recursive

# Install the gripper package
pip install -e robotiq_2f_85_controller/
```

Then start the RobotEnv server with `--gripper-type robotiq`:

```shell
python src/dexcontrol/core/robotenv_vega/server.py \
    --arm-side left \
    --gripper-type robotiq \
    --robotiq-comport /dev/ttyUSB0
```

Or instantiate `VegaRobot` directly:

```python
from dexcontrol.core.vega.robot import VegaRobot

robot = VegaRobot(
    arm_side="left",
    gripper_type="robotiq",
    robotiq_comport="/dev/ttyUSB0",
)
```

## ⚠️ Version Compatibility

**Important:** `dexcontrol >= 0.4.0` requires robot firmware `>= 0.4.0`. Using older firmware with this version will not work.

> **Note:** `dexcontrol 0.4.x` depends on `dexcomm >= 0.4.0`, which is **not compatible** with `dexcontrol 0.3.x`. If you need to stay on `dexcontrol 0.3.x`, do not upgrade `dexcomm` to `0.4.0` or above.

**Before upgrading, check your current firmware version:**
```shell
dextop firmware info
```

If your firmware is outdated, please update it before installing the new version to ensure full compatibility. Please contact the Dexmate team if you do not know how to do it.

**📋 See [CHANGELOG.md](./CHANGELOG.md) for detailed release notes and version history.**

## 📄 Licensing

This project is **dual-licensed**:

### 🔓 Open Source License
This software is available under the **GNU Affero General Public License v3.0 (AGPL-3.0)**.
See the [LICENSE](./LICENSE) file for details.

### 💼 Commercial License
For businesses that want to use this software in proprietary applications without the AGPL requirements, commercial licenses are available.

**📧 Contact us for commercial licensing:** contact@dexmate.ai

**Commercial licenses provide:**
- ✅ Right to use in closed-source applications
- ✅ No source code disclosure requirements
- ✅ Priority support options


## 📚 Examples

Explore our comprehensive examples in the `examples/` directory:

- 🎮 **Basic Control** - Simple movement and sensor reading
- 🎯 **Advanced Control** - Complex manipulation tasks
- 📺 **Teleoperation** - Remote control interfaces
- 🔧 **Troubleshooting** - Diagnostic and maintenance tools

---

<div align="center">
  <h3>🤝 Ready to build amazing robots?</h3>
  <p>
    <a href="mailto:contact@dexmate.ai">📧 Contact Us</a> •
    <a href="./examples/">📚 View Examples</a> •
  </p>
</div>
