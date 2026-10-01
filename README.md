# Quaternion Library

A C (C99) quaternion library for embedded firmware (STM32 / UAV), designed for attitude estimation (AHRS / ESKF). The source code aims for compliance with **MISRA-C** and **NASA Power of Ten** rules: no dynamic memory allocation, no recursion, no unbounded loops, and parameter validation using `assert`.

* **Author:** vultra_dev

* **Date Created:** May 24, 2026

* **Convention:** $q = w + xi + yj + zk$, Hamilton quaternion, 32-bit `float` type

## 1. Directory Structure

| File | Role | 
 | ----- | ----- | 
| `Quaternion.h` | Data types, constants, `static inline` functions, and public function declarations | 
| `Quaternion.c` | Implementation of non-inline functions | 
| `main.c` | Test program for Frama-C/Eva (boundary value ranges) | 
| `Frama_C_Test` | Saved Frama-C session (gzip compressed file) | 
| `open_frama.sh` | Opens session in Ivette: `ivette -load Frama_C_Test` | 

## 2. Data Types and Constants

```
typedef struct { float w; float x, y, z; } Quaternion_t;   /* w: real part; x, y, z: imaginary parts */
typedef struct { float roll, pitch, yaw; } Euler_t;        /* units in radians, rotating around X, Y, Z axes */

```

| Constant | Value | Meaning | 
 | ----- | ----- | ----- | 
| `QUAT_EPS` | `1e-6f` | Threshold for protection against division by near-zero values | 
| `HALF_PI` | `1.570793f` | $\pi/2$, used when pitch encounters gimbal lock ($\pm 90^\circ$) | 

## 3. API

### 3.1 `static inline` Functions (in header)

| Function | Description | 
 | ----- | ----- | 
| `Quat_Create(w, x, y, z)` | Creates a quaternion from 4 components | 
| `Quat_Identity()` | Returns $1 + 0i + 0j + 0k$ | 
| `Quat_Add(q1, q2)` | Component-wise addition | 
| `Quat_Scale(q, s)` | Scalar multiplication $s \cdot q$ | 
| `Quat_NormSq(q)` | Squared norm: $w^2 + x^2 + y^2 + z^2$ | 
| `Conjugation(q)` | Conjugate quaternion: $w - xi - yj - zk$ | 

### 3.2 Functions in `Quaternion.c`

| Function | Description | 
 | ----- | ----- | 
| `void Quat_Multiply(const Quaternion_t *q1, const Quaternion_t *q2, Quaternion_t *res)` | Hamilton product `res = q1 * q2` (non-commutative). `res` **is allowed to alias** `q1` or `q2` because input components are stored temporarily before writing | 
| `void Quat_Normalize(Quaternion_t *q)` | In-place normalization to $\Vert{}q\Vert{} = 1$. If $\Vert{}q\Vert{}$ is too small, **`q` remains unchanged** (not set to zero, as a zero quaternion is not a valid attitude) | 
| `void Quat_Reciprocal(const Quaternion_t *q, Quaternion_t *res)` | Reciprocal (inverse) $q^{-1} = q^* / \Vert{}q\Vert{}^2$. For unit quaternions, this equals the conjugate | 
| `void Quat_ToEuler(Quaternion_t *q, Euler_t *angle)` | Converts a unit quaternion to ZYX Euler angles (roll-pitch-yaw), with gimbal lock clamping | 
| `void Quat_RotateVector(const Quaternion_t *q, const float v_in[3], float v_out[3])` | Rotates a 3D vector using a unit quaternion via an optimized Rodrigues' formula (no full Hamilton product required) | 

### 3.3 Formulas

**Hamilton Product**

$$
\begin{aligned}  w &= w_1 w_2 - x_1 x_2 - y_1 y_2 - z_1 z_2 \\  x &= w_1 x_2 + x_1 w_2 + y_1 z_2 - z_1 y_2 \\  y &= w_1 y_2 - x_1 z_2 + y_1 w_2 + z_1 x_2 \\  z &= w_1 z_2 + x_1 y_2 - y_1 x_2 + z_1 w_2  \end{aligned}
$$

**Quaternion → Euler (ZYX, aerospace convention)**

$$
\begin{aligned}  \text{roll } \phi &= \text{atan2}(2(w \cdot x + y \cdot z), w^2 - x^2 - y^2 + z^2) \\  \text{pitch } \theta &= \text{asin}(2(w \cdot y - z \cdot x)) \\  \text{yaw } \psi &= \text{atan2}(2(w \cdot z + x \cdot y), w^2 + x^2 - y^2 - z^2)  \end{aligned}
$$

*(Note: pitch* $\theta$ *is clamped to* $\pm \pi/2$ *when* $\vert{}\sin\theta\vert{} \ge 1$*)*

**Vector Rotation (Rodrigues' form)**

$$
\begin{aligned}  \mathbf{t} &= 2 \cdot (\mathbf{q}_{\text{vec}} \times \mathbf{v}_{\text{in}}) \\  \mathbf{v}_{\text{out}} &= \mathbf{v}_{\text{in}} + w \cdot \mathbf{t} + (\mathbf{q}_{\text{vec}} \times \mathbf{t})  \end{aligned}
$$

### 3.4 Division-by-Zero Protection

The internal function `save_inv(x)` (`static`, inside `Quaternion.c`) centralizes protection against near-zero division: returns $1/x$ if $x > \text{QUAT\_EPS}$, otherwise returns $0$. Both `Quat_Normalize` (via `sqrtf(NormSq)`) and `Quat_Reciprocal` (via `NormSq`) utilize this function.

## 4. Usage Example

```
#include "Quaternion.h"

Quaternion_t q   = Quat_Create(0.7071f, 0.0f, 0.7071f, 0.0f);
Quaternion_t dq  = Quat_Identity();
Quaternion_t out;
Euler_t      eul;
float v[3]  = {1.0f, 0.0f, 0.0f};
float vr[3];

Quat_Normalize(&q);
Quat_Multiply(&q, &dq, &out);   /* out = q * dq */
Quat_ToEuler(&out, &eul);       /* eul.roll / pitch / yaw [rad] */
Quat_RotateVector(&q, v, vr);   /* vr = q ⊗ v ⊗ q⁻¹ */

```

## 5. Build

The library requires only `<stdio.h>`, `<math.h>`, `<assert.h>` and linking with `libm`.

```
gcc -std=c99 -Wall -Wextra -Wpedantic -c Quaternion.c -o Quaternion.o

```

For STM32, add `Quaternion.c` to your project and enable the single-precision FPU (`-mfpu=fpv4-sp-d16 -mfloat-abi=hard` on STM32F4).

> `assert` statements will be removed when building with `-DNDEBUG`. If the release build needs to retain parameter checking, replace them with a custom error-handling mechanism.

## 6. Code Review and Testing Workflow

The workflow consists of 4 steps, progressing from light to rigorous.

### Step 1 – Manual Checklist Review

* \[ \] All input pointers have `assert(ptr != NULL)` (Power of Ten, rule 5)

* \[ \] No dynamic memory allocation, recursion, or `goto`; no unbounded loops

* \[ \] All divisions and `sqrtf` / `asinf` / `atan2f` operations have domain protection

* \[ \] Functions supporting aliasing (`res == q1`) properly buffer inputs beforehand

* \[ \] Functions that do not modify inputs declare them as `const`

* \[ \] Doc comments in the header match implementation (see section 7)

* \[ \] Clean compilation with `-Wall -Wextra -Wpedantic -Wconversion -Wdouble-promotion`

### Step 2 – Static Analysis with Frama-C / Eva

The test program `main.c` uses `Frama_C_float_interval()` to generate **continuous value ranges** instead of a few discrete sample points:

| Variable | Range | 
 | ----- | ----- | 
| `q1`, `q2` (w, x, y, z) | `[-2.0, 2.0]` (passes through 0 to test degenerate cases) | 
| `v_in[3]` | `[-10.0, 10.0]` | 

Execution flow: `Quat_Normalize(&q1)` → `Quat_Multiply(&q1, &q2, &res)` → `Quat_ToEuler(&res, &angle)` → `Quat_RotateVector(&q1, v_in, v_out)`.

Run from command line:

```
frama-c -eva -main main -warn-special-float non-finite \
        main.c Quaternion.c -save Frama_C_Test

```

Reopen saved results in the graphical interface:

```
./open_frama.sh        # equivalent to: ivette -load Frama_C_Test

```

The saved session uses Eva's default configuration (`cvalue` domain) with `-warn-special-float non-finite`, so Eva warns when a float operation might result in **NaN or** $\pm\text{Inf}$. Frama-C is installed via opam (OCaml 4.14.1).

How to read results in Ivette:

1. Open the **Properties** panel and filter by `Unknown` / `Invalid` status.

2. For each alarm, determine its category:

   * Floating-point multiplication/addition overflow / NaN / Inf

   * Domain range for `asinf`, `atan2f`, `sqrtf`

   * User assertion (`assert(x >= 0.0f)` in `save_inv`)

3. Classify: real bug (fix code) or false alarm due to overly broad input ranges (narrow ranges or add preconditions).

4. Record conclusions in the table in section 8.

For higher Eva precision (fewer false alarms), try `-eva-precision 3` or increase `-eva-slevel`.

### Step 3 – Unit Testing on PC

Compare against reference implementations (NumPy / `scipy.spatial.transform.Rotation`) for the following cases:

| Case | Expected Result | 
 | ----- | ----- | 
| `Quat_Identity` through all functions | Results remain unchanged | 
| $\Vert{}q\Vert{} < \text{QUAT\_EPS}$ through `Quat_Normalize` | `q` remains unchanged | 
| Unit $q$, `Quat_Reciprocal` | Equals `Conjugation(q)` | 
| `res` aliases `q1` in `Quat_Multiply` | Matches result when `res` is separate | 
| Pitch = $\pm 90^\circ$ (gimbal lock) | No NaN, pitch equals $\pm\text{HALF\_PI}$ | 
| Rotate vector then rotate back by $q^{-1}$ | Returns to original vector (within float precision tolerance) | 

### Step 4 – Hardware Testing

Run on STM32F407 with real IMU data, comparing against PC results to detect discrepancies caused by single-precision FPU hardware implementation.

## 7. Review Notes (Discovered During Code Inspection)

| \# | Location | Issue | Proposal | 
 | ----- | ----- | ----- | ----- | 
| 1 | `Quaternion.h`, doc of `Quat_ToEuler` | Roll formula documented as $2(wy + xz)$, but code uses $2(wx + yz)$ | Update documentation to match code | 
| 2 | `HALF_PI = 1.570793f` | Exact $\pi/2$ is `1.5707964f`; current value differs by $\sim 3 \times 10^{-7}\text{ rad}$ (and is truncated further as float) | Use `1.5707964f` | 
| 3 | Doc of `Quat_Reciprocal` | Documents precondition $\Vert{}q\Vert{} > \text{QUAT\_EPS}$, but code checks $\Vert{}q\Vert{}^2 > \text{QUAT\_EPS}$ (i.e. $\Vert{}q\Vert{} > 10^{-3}$) and returns zero quaternion if unmet | Align threshold and clearly state behavior upon degeneracy | 
| 4 | Comment in `save_inv` | Says "Fast inverse square-root", but the function is just protected inversion | Fix comment | 
| 5 | `Quat_ToEuler` | Parameter `q` is not `const`, even though the function does not modify `q` | Change to `const Quaternion_t *q` | 
| 6 | `Conjugation` | Lacks `Quat_` prefix unlike other functions | Rename to `Quat_Conjugate` | 
| 7 | `Quat_Normalize` | Does not notify caller when skipped due to an excessively small norm | Consider returning a status code | 
| 8 | `Quaternion.h` | Unnecessary inclusion of `<stdio.h>`; `NULL` should come from `<stddef.h>` | Remove `<stdio.h>`, add `<stddef.h>` | 
| 9 | `static const float` in header | Each translation unit gets a private copy; MISRA typically prefers `#define` or `enum` for constants | Consider changing | 
| 10 | `main.c` | Does not call `Quat_Reciprocal` and `Conjugation`; `q2` is not normalized before multiplication | Add to test suite | 

## 8. Review Results Table

| Date | Reviewer | Tool | Result | Action Items | 
 | ----- | ----- | ----- | ----- | ----- | 
| 01/10/2026 | DEv-vUltra | Frama-C | Pass |  | 

## 9. Known Limitations

* `Quat_ToEuler` and `Quat_RotateVector` assume the quaternion **is already normalized**; the functions do not perform auto-verification.

* Precision is 32-bit `float`; accumulated error requires periodic re-normalization inside estimation loops.

* Euler angles have a singularity at pitch = $\pm 90^\circ$; use quaternions for internal calculations and convert to Euler only for display or logging.