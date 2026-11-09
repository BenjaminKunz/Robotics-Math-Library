# 🤖 Robotics Math Library

A lightweight Python library for robotics mathematics — rotation matrices, Denavit-Hartenberg transformations, forward kinematics, and vector operations — built on top of NumPy.

---

## 📦 Installation

No separate package needed. Simply copy the module into your project and install the dependency:

```bash
pip install numpy
```

Then import the module:

```python
import robotics_math as rm  # rename the file as needed
```

---

## 🚀 Quick Start

```python
import numpy as np
from robotics_math import yaw_pitch_roll, DH_Matrix, forward_kinematics, Vektor

# Rotation matrix for 90° yaw
R = yaw_pitch_roll(yaw_grad=90, pitch_grad=0, roll_grad=0)
print(R)

# Rotate a point
p = np.array([1, 0, 0])
p_rotated = yaw_pitch_roll(90, 0, 0, p=p)
print(p_rotated)

# Vector operations
v1 = Vektor([1, 2, 3])
v2 = Vektor([4, 5, 6])
print(v1.winkel(v2))   # Angle in degrees
```

---

## 📚 API Reference

### `yaw_pitch_roll(yaw_grad, pitch_grad, roll_grad, p=None)`

Computes a 3×3 rotation matrix using the **ZYX (intrinsic) convention** (Yaw → Pitch → Roll).

| Parameter | Type | Description |
|---|---|---|
| `yaw_grad` | `float` | Yaw angle in degrees (rotation around Z-axis) |
| `pitch_grad` | `float` | Pitch angle in degrees (rotation around Y-axis) |
| `roll_grad` | `float` | Roll angle in degrees (rotation around X-axis) |
| `p` | `np.ndarray` or `None` | Optional 3D point to rotate. If `None`, returns the rotation matrix. |

**Returns:** `np.ndarray` — either the 3×3 rotation matrix `R`, or the rotated point `R @ p`.

```python
R = yaw_pitch_roll(45, 30, 15)         # → 3×3 rotation matrix
p_new = yaw_pitch_roll(45, 30, 15, p=np.array([1, 0, 0]))  # → rotated point
```

---

### `cosinus_satz(ak, bk, gk)`

Computes the angle **γ** (gamma) opposite to side `gk` using the **law of cosines**.

| Parameter | Type | Description |
|---|---|---|
| `ak` | `float` | Length of side a |
| `bk` | `float` | Length of side b |
| `gk` | `float` | Length of side c (opposite to angle γ) |

**Returns:** `float` — angle γ in radians.

```python
gamma = cosinus_satz(3, 4, 5)  # right triangle → π/2
```

---

### `DH_Matrix(alpha, a, theta, d)`

Computes a **4×4 Denavit-Hartenberg transformation matrix**.

| Parameter | Type | Description |
|---|---|---|
| `alpha` | `float` | Link twist (radians) |
| `a` | `float` | Link length |
| `theta` | `float` | Joint angle (radians) |
| `d` | `float` | Link offset |

**Returns:** `np.ndarray` — 4×4 homogeneous transformation matrix.

```python
T = DH_Matrix(alpha=0, a=1.0, theta=np.pi/2, d=0)
```

---

### `forward_kinematics(DH_params)`

Computes **forward kinematics** for a serial robot arm by chaining DH matrices.

| Parameter | Type | Description |
|---|---|---|
| `DH_params` | `np.ndarray` | Shape `(n, 4)` — each row is `[alpha, a, theta, d]` for one joint |

**Returns:** `list` of `np.ndarray` — list of `n+1` transformation matrices (including the base frame).

```python
DH_params = np.array([
    [0,    0,   np.pi/2, 0.5],
    [0,    1.0, 0,       0  ],
    [0,    0.8, 0,       0  ],
])
frames = forward_kinematics(DH_params)
end_effector = frames[-1]
```

---

### `class Vektor`

A 3D vector utility class wrapping NumPy for common robotics vector operations.

#### Constructor

```python
v = Vektor([x, y, z])
```

#### Properties

| Property | Description |
|---|---|
| `.betrag` | Euclidean magnitude (length) of the vector |
| `.einheitsvektor` | Unit vector (normalized) |

#### Methods

| Method | Description |
|---|---|
| `.skalarprodukt(other)` | Dot product with another `Vektor` |
| `.kreuzprodukt(other)` | Cross product with another `Vektor` |
| `.winkel(other)` | Angle between two vectors in **degrees** |
| `.info()` | Prints vector, magnitude, and unit vector |

```python
v1 = Vektor([1, 0, 0])
v2 = Vektor([0, 1, 0])

print(v1.betrag)              # 1.0
print(v1.winkel(v2))          # 90.0
print(v1.kreuzprodukt(v2))    # [0, 0, 1]
v1.info()
```

---

### `XYZ_RotationsMatrix(achse, winkel)`

Returns a **3×3 rotation matrix** for a single-axis rotation.

| Parameter | Type | Description |
|---|---|---|
| `achse` | `str` | Axis: `'X'`, `'Y'`, or `'Z'` (case-insensitive) |
| `winkel` | `float` | Rotation angle in **radians** |

**Returns:** `np.ndarray` — 3×3 rotation matrix.

```python
R = XYZ_RotationsMatrix('Z', np.pi/4)  # 45° rotation around Z
```

---

### `Transformationsmatrix(achse, winkel, vektor)`

Builds a **4×4 homogeneous transformation matrix** from a rotation axis, angle, and translation vector.

| Parameter | Type | Description |
|---|---|---|
| `achse` | `str` | Rotation axis: `'X'`, `'Y'`, or `'Z'` |
| `winkel` | `float` | Rotation angle in radians |
| `vektor` | `np.ndarray` | Shape `(4, 1)` — translation vector as homogeneous column |

**Returns:** `np.ndarray` — 4×4 transformation matrix.

```python
t = np.array([[1], [0], [0], [1]])
T = Transformationsmatrix('Z', np.pi/2, t)
```

---

### `invers_transformationsmatrix(T)`

Computes the **inverse of a 4×4 homogeneous transformation matrix** efficiently (using `R⁻¹` and `-R⁻¹ · t`).

| Parameter | Type | Description |
|---|---|---|
| `T` | `np.ndarray` | 4×4 homogeneous transformation matrix |

**Returns:** `np.ndarray` — 4×4 inverse transformation matrix.

```python
T_inv = invers_transformationsmatrix(T)
```

---

## 🧪 Examples

### Full Forward Kinematics Pipeline

```python
import numpy as np
from robotics_math import DH_Matrix, forward_kinematics

# 3-DOF planar robot arm
DH_params = np.array([
    [0, 1.0, np.pi/4, 0],
    [0, 0.8, np.pi/6, 0],
    [0, 0.5, 0,       0],
])

frames = forward_kinematics(DH_params)

for i, T in enumerate(frames):
    print(f"Frame {i}:")
    print(T)
    print()
```

### Rotation with Yaw-Pitch-Roll

```python
from robotics_math import yaw_pitch_roll
import numpy as np

# Rotate point [1, 0, 0] by 90° yaw
p = np.array([1, 0, 0])
p_new = yaw_pitch_roll(90, 0, 0, p=p)
# Result ≈ [0, 1, 0]
print(p_new)
```

### Inverse Kinematics Helper (Law of Cosines)

```python
from robotics_math import cosinus_satz
import numpy as np

# Two-link arm: l1=1.0, l2=0.8, target distance=1.2
l1, l2, d = 1.0, 0.8, 1.2
elbow_angle = cosinus_satz(l1, l2, d)
print(f"Elbow angle: {np.degrees(elbow_angle):.2f}°")
```

---

## 🔧 Dependencies

| Package | Version |
|---|---|
| `numpy` | ≥ 1.21 |

---

## 📄 License

MIT License — feel free to use, modify, and distribute.
