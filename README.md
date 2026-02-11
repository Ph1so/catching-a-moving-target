# catching-a-moving-target

## Strategy 1: Trajectory Endpoint Intercept
Perform **A* Search** on the last node of the target trajectory. Once the robot reaches the terminal node, it follows the path until it intercepts the target.

### Performance Comparison

| Metric | Baseline Run | Baseline Run 2|
| :--- | :---: | :---: |
| **Visual** | <img src="baseline_2.gif" width="300"> | <img src="myGIF.gif" width="300"> |
| **Target Caught** | 1 | 0 |
| **Target Steps** | 2,882 | 5,245 |
| **Time Taken (s)** | 2,882 | 5,244 |
| **Moves Made** | 2,882 | 3,958 |
| **Path Cost** | 2,882 | 8,306,796 |

---
