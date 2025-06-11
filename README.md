# Localization Node Overview

This section explains, in simple terms, how the `fastslamwithekf.py` node tracks the car’s position and builds a map of cones.

## 0. What this node *is* and *why* it exists

* **Goal:**

  1. **Track** where the car is on the track at every moment.
  2. **Build a map** of the cones the car’s camera sees.
* **How:**

  * Keep a *swarm* of guesses about the car’s position (called **particles**).
  * Move each guess forward when the car drives (the **bicycle model**).
  * Correct those guesses whenever the camera sees cones (an **EKF update**).

## 1. ROS basics: node, topics, subscribers, publishers

```python
rospy.init_node("uKalman_Filter_node")
self.cones_sub   = rospy.Subscriber("/camera_cones_marker", MarkerArray, self.cones_callback)
self.control_sub = rospy.Subscriber("vel_ekf", Float32MultiArray,   self.control_callback)
self.path_pub    = rospy.Publisher("/robot_path", Path, queue_size=1)
```

* **Node:** A running program in ROS named `uKalman_Filter_node`.
* **Topics:** Named channels for data flow:

  * `/camera_cones_marker`: cone detections from vision.
  * `/vel_ekf`: car’s forward & sideways speed plus heading.
  * `/robot_path`: where we publish our estimated trajectory.
* **Subscriber:** Listens to a topic and calls a function when data arrives.
* **Publisher:** Sends messages to a topic for others (or RViz) to use.

## 2. Particles: a swarm of “where‑am‑I” guesses

```python
class Particle:
    def __init__(self, x, y, yaw):
        self.x, self.y, self.yaw = x, y, yaw               # position guess
        self.w = 1.0 / N_PARTICLE                          # initial confidence
        self.lm = {}    # this guess’s personal map of cones
```

* Think of **50 friends**, each writing down a guess of the car’s position `(x, y)` and orientation `yaw`.
* `w` is their *confidence*—initially equal for all.
* Each friend also keeps their own *map of cones* (`lm`), starting empty.

## 3. Moving guesses forward: the **bicycle model**

```python
def bicyclemodel(self, particle, dt):
    # 1. Get measured speeds (vx forward, vy sideways)
    # 2. Rotate them into world‑coordinates using heading (yaw)
    vx_w = self.vx * cos(yaw) - self.vy * sin(yaw)
    vy_w = self.vx * sin(yaw) + self.vy * cos(yaw)

    # 3. Move each guess by speed × time (with a bit of noise)
    particle.x += (vx_w + small_noise) * dt
    particle.y += (vy_w + small_noise) * dt

    # 4. Snap its heading to the latest measured yaw
    particle.yaw = yaw
    return particle
```

* \`\` come from `/vel_ekf` (speed in the car’s own frame).
* We **rotate** these into the world frame (so forward always goes where the car faces).
* Then we **step** each guess by `speed × dt`, adding a bit of randomness (noise).

### 3a. Tiny example

* Friend A thinks the car is at `(0,0)` pointing north (90°).
* The car reports `vx=2 m/s`, `vy=0`, `yaw=90°`, and `dt=1 s`.
* Rotate `(2,0)` by 90° → `(0,2)` in world frame, then add to `(0,0)` → `(0,2)`.
* So Friend A now thinks the car is at `(0,2)`, still facing north.

## 4. Seeing cones: the **EKF map**

When the camera detects cones, it gives positions *relative* to the car. We must:

1. **Translate** each cone into world‑coordinates using our best guess of `(x,y,yaw)`.
2. **Decide** if it’s a cone we’ve already mapped or a brand‑new one.
3. If **old**, run a tiny Kalman‑filter update on every guess’s copy of that cone and adjust each guess’s confidence based on how well it matched.
4. If **new**, add it into every guess’s map with initial uncertainty.

### EKF in plain words

* **EKF = Extended Kalman Filter**: a method to merge what you *expected* to see (old map) with what you *actually* saw.
* For each cone, you keep:

  * A **mean** (its estimated position).
  * A **covariance** (how unsure you are).
* **Update step**:

  * Compare predicted cone position vs. detected position.
  * Compute a correction to nudge the mean and shrink the uncertainty.
  * Use that correction to *reward* guesses whose map matches the detection, and *penalize* those that don’t.

## 5. Putting it together: **Predict → Update** loop

1. **Predict (every 0.05 s):**

   * Apply the **bicycle model** to move each guess.

2. **Update (when cones arrive):**

   1. Compute the overall best guess of the car’s pose (weighted average of all guesses).
   2. Split cone detections into “old” vs. “new.”
   3. For each old cone:

      * Run the EKF update on each guess’s cone.
      * Multiply each guess’s confidence by how well it matched.
   4. Normalize confidences; if too few *effective* guesses remain, **resample** (drop bad guesses and duplicate good ones).
   5. Add brand‑new cones into every guess’s map.
   6. Recompute the best‑guess pose.

3. **Visualize:**

   * Append the best‑guess pose to `/robot_path`.
   * Publish little dots for each guess (particles).
   * Publish spheres for each cone in the map.
   * Publish a big sphere at the best‑guess pose.

## 6. Why this works

* **Many guesses** ensure at least some follow the true path.
* **Noise + resampling** let the filter discard bad guesses and reinforce good ones.
* **EKF cone updates** correct the swarm whenever you spot known landmarks.
* **Over time**, the swarm converges on the true path, and the map of cones becomes accurate.

### Final, super‑simple analogy

> Imagine **50 blindfolded friends** standing in a dark field full of traffic cones. You drive a remote‑controlled car around:
>
> 1. Each friend writes down a guess of the car’s position and faces forward.
> 2. You tell them “move forward 1 m, turn slightly right,” and they each move their guess—but wobble a bit (noise).
> 3. When the car’s camera sees a cone, you shout “there’s a cone 5 m ahead and 2 m to the left!”
>
>    * Friends whose own sketch of cones matches your shout get more confident.
>    * Those whose maps disagree get less confident.
>    * If it’s a new cone, everyone adds it.
> 4. Every so often, the least confident friends copy the guesses of the most confident ones (resampling).
> 5. As you drive around, your group’s guesses cluster tightly on the car’s true path, and their cone map fills in accurately.

That is **fastSLAM with an EKF map** in the simplest terms!

## Visualization Setup

Launch the localization node together with RViz:

```bash
$ roslaunch AAM_LOCALIZTION localization.launch
```

RViz will show:

* **TF** tree of `map` and `odom` frames.
* **Path** display on `/robot_path` in bright green.
* **MarkerArray** displays for `/particles_pub` and `/cone_loc`.
* **Marker** display for `/visualization_loc`.
* **Vector** display for `/velocity_vectors` showing the current velocity arrows.
