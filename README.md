# Science Olympiad Robot Tour

This code is for my Science Olympiad Robot Tour robot, and uses pure-pursuit and PID to accurately reach its waypoints and complete on time.

## Installation

### Robot Code

1. Clone the repo

```bash
git clone https://github.com/krishwu/SciOlyRobot.git
```

2. Open in Arduino IDE

3. Install libraries

- _Encoder_ by Paul Stroffregen
- _L298N_ by Andrea Lombardo
- _PID_ by Brett Beauregard
- _SparkFun BNO08x Cortex Based IMU_ by SparkFun Electronics

4. Upload code

### Visualization

1. Create virtual environment

```bash
cd serialVisualization
python3 -m venv .venv
pip3 install -r requirements.txt
```

2. Run code

```bash
python3 runVisualization.py
```

Made with ❤️ by [@KrishWu](https://www.github.com/KrishWu)
