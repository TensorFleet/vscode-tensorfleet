# SO-ARM101 Quick Start

Follow this guide any time you work with the SO-ARM101 example. The first section covers the simulated keyboard teleop you have today, and the second introduces the leader arm workflow that streams real controller input into the simulated robot.

## 1. Keyboard teleop (simulated arm)

1. **Start VM**: Click the **TensorFleet** status bar in VS Code and select **Start VM**.
2. **Setup Environment**: Install dependencies using `uv`.
   ```bash
   uv venv && uv pip install -r requirements.arm.txt
   ```
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

## 2. Leader arm teleop (real input into simulated arm)

This mode injects live rotations from a real SO101 leader arm into the simulator. Keep clear notes of the USB ports you use so you can reuse them later (save the port path in your notes or `.env` file). If you disconnect/reconnect cables, repeat the port discovery step.

### A. Find the leader arm USB port

1. Activate the Python environment:
   ```bash
   source .venv/bin/activate
   ```
2. Run the port discovery helper:
   ```bash
   lerobot-find-port
   ```

   **Example output:**
   ```
   Finding all available ports for the MotorBus.
   ['/dev/tty.usbmodem575E0032081', '/dev/tty.usbmodem575E0031751']
   Remove the USB cable from your MotorBus and press Enter when done.

   [...Disconnect corresponding leader or follower arm and press Enter...]

   The port of this MotorBus is /dev/tty.usbmodem575E0032081
   Reconnect the USB cable.
   ```

   > Tip: jot down the port you plan to reuse (e.g., `/dev/ttyACM1`) so calibration and teleop steps stay consistent.

### B. Calibrate the leader arm

Follow the Hugging Face calibration guide (includes video): https://huggingface.co/docs/lerobot/en/so101  

Then run:

```bash
lerobot-calibrate --teleop.type=so101_leader --teleop.port=/dev/ttyACM0 --teleop.id=my_awesome_leader_arm
```

Replace `--teleop.port` with the port you saved and `--teleop.id` with a memorable name. If calibration data lives outside the default cache, add `--calibration-dir /path/to/calibration/teleoperators/so_leader`.

### C. Run teleoperation

Pick the script that matches your desired input:

- **Simulated arm via keyboard (default, still available)**:
  ```bash
  uv run python src/teleop_so_arm101.py --input keyboard
  ```
  The simulator resets to the starting (home) position each time the script launches.

- **Simulated arm driven by leader arm input (lowest latency)**:
  ```bash
  uv run python src/teleop_so_arm101.py --input leader --leader-port /dev/ttyACM1 --leader-id my_awesome_leader_arm
  ```
  Swap `/dev/ttyACM1` and `my_awesome_leader_arm` with the port and nickname you recorded earlier. Pass `--calibration-dir /path/to/calibration/teleoperators/so_leader` if needed.

Feel free to reconnect the leader arm and rerun the leader command whenever you need live control; the simulator reads from the saved teleop ID and port every time.
