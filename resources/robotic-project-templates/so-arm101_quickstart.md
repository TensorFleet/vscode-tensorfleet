# SO-ARM101 Quick Start

1. **Start VM**: Click the **TensorFleet** status bar in VS Code and select **Start VM**.
2. **Setup Environment**: Install dependencies using `uv`.
   ```bash
   uv venv && uv pip install -r requirements.arm.txt
   ```
   > [!NOTE]
   > Hardware-only packages are now limited to Linux installs. macOS/Windows users can run the command above without hitting missing `torchvision` wheels.
3. **Open Simulation**: Launch the **Simulation View** and zoom in on the table as shown.
   ![Initial View](assets/simulation_view_01.png)
   ![Zoomed View](assets/simulation_view_02.png)
4. **Monitor Camera Feeds**: Open several **Image Panels** and subscribe to these camera topics:
   - `/so_arm101/agent_camera/image_raw`
   - `/so_arm101/side_camera/image_raw`
   - `/so_arm101/wrist_camera/image_raw`
   ![Image Panels](assets/image_panels.png)

   **Tip: Using Raw Messages**
   For a more compact view of multiple topics, use the **Raw Messages** panel:
   - Open the **Raw Messages** panel.
   - Search for your topic in **Search Topics**.
   - Select the topic and click **Subscribe**.

   ![Raw Messages](assets/raw_messages.png)

5. **Run Teleoperation**: Start the keyboard control script:
   ```bash
   uv run python src/teleop_so_arm101.py
   ```
   > [!IMPORTANT]
   > Keyboard controls will only work when the **terminal window is focused**.

### Keyboard Controls

| Key | Action |
| --- | --- |
| `q` / `a` | Joint 1 +/- |
| `w` / `s` | Joint 2 +/- |
| `e` / `d` | Joint 3 +/- |
| `r` / `f` | Joint 4 +/- |
| `t` / `g` | Joint 5 +/- |
| `y` / `h` | Gripper open/close |
| `space` | Hold current position |
| `0` | Return to home position |
| `x` / `Ctrl-C` | Exit teleoperation |


## Using a leader arm

1. Find the USB port for the leader arm:

```
source .venv/bin/activate
```

```
lerobot-find-port
```
Example Output:
```
Finding all available ports for the MotorBus.
['/dev/tty.usbmodem575E0032081', '/dev/tty.usbmodem575E0031751']
Remove the USB cable from your MotorsBus and press Enter when done.

[...Disconnect corresponding leader or follower arm and press Enter...]

The port of this MotorsBus is /dev/tty.usbmodem575E0032081
Reconnect the USB cable.
```

2. Calibrate the leader arm  
https://huggingface.co/docs/lerobot/en/so101 (Calibration video)

```
lerobot-calibrate     --teleop.type=so101_leader     --teleop.port=/dev/ttyACM0 --teleop.id=my_awesome_leader_arm
```

3. Run the script (pick one):

- Simulated arm via keyboard (default):
  ```
  uv run python src/teleop_so_arm101.py --input keyboard
  ```
  The simulator resets to the starting (home) position when the script starts.

- Simulated arm driven by leader input (lowest latency):
  ```
  uv run python src/teleop_so_arm101.py --input leader --leader-port /dev/ttyACM1 --leader-id my_awesome_leader_arm
  ```
If your calibration files live outside the default cache location, pass `--calibration-dir /path/to/calibration/teleoperators/so_leader`.
