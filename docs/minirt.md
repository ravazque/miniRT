
# miniRT - Technical Documentation

This document provides comprehensive technical information about the raytracing implementation, mathematical foundations, and key algorithms used in the miniRT project.

---

## Table of Contents

1. [Ray Tracing Fundamentals](#1-ray-tracing-fundamentals)
2. [Vector Mathematics](#2-vector-mathematics)
3. [Ray Generation](#3-ray-generation)
4. [Ray-Object Intersections](#4-ray-object-intersections)
5. [Lighting Model](#5-lighting-model)
6. [Shadow Computation](#6-shadow-computation)
7. [MiniLibX Graphics](#7-minilibx-graphics)
8. [Data Structures](#8-data-structures)

---

## 1. Ray Tracing Fundamentals

### What is Ray Tracing?

Ray tracing is a rendering technique that simulates the physical behavior of light to generate realistic images. For each pixel in the output image, we cast a ray from the camera through that pixel into the 3D scene and determine what color that ray "sees."

### Basic Algorithm

```
for each pixel (x, y) in image:
    ray = generate_ray(camera, x, y)
    hit = find_closest_intersection(ray, scene)
    if hit:
        color = calculate_lighting(hit, scene)
    else:
        color = background_color
    set_pixel(x, y, color)
```

### Ray Definition

A ray is defined by an origin point **O** and a direction vector **D**:

```
P(t) = O + t * D    where t >= 0
```

- **O**: Origin point (camera position or shadow ray origin)
- **D**: Normalized direction vector
- **t**: Parameter determining distance along the ray
- **P(t)**: Point on the ray at parameter t

---

## 2. Vector Mathematics

### 3D Vector Structure

```c
typedef struct s_vec3
{
    double x;
    double y;
    double z;
} t_vec3;
```

### Essential Operations

#### Vector Creation
```c
t_vec3 vec3_new(double x, double y, double z)
{
    t_vec3 v;
    v.x = x;
    v.y = y;
    v.z = z;
    return (v);
}
```

#### Addition & Subtraction
```c
// A + B = (Ax + Bx, Ay + By, Az + Bz)
t_vec3 vec3_add(t_vec3 a, t_vec3 b)
{
    return vec3_new(a.x + b.x, a.y + b.y, a.z + b.z);
}

// A - B = (Ax - Bx, Ay - By, Az - Bz)
t_vec3 vec3_sub(t_vec3 a, t_vec3 b)
{
    return vec3_new(a.x - b.x, a.y - b.y, a.z - b.z);
}
```

#### Scalar Multiplication
```c
// s * V = (s*Vx, s*Vy, s*Vz)
t_vec3 vec3_scale(t_vec3 v, double s)
{
    return vec3_new(v.x * s, v.y * s, v.z * s);
}
```

#### Dot Product

The dot product is fundamental for angle calculations and projections:

```
A · B = Ax*Bx + Ay*By + Az*Bz = |A| * |B| * cos(θ)
```

```c
double vec3_dot(t_vec3 a, t_vec3 b)
{
    return (a.x * b.x + a.y * b.y + a.z * b.z);
}
```

**Applications:**
- Calculating angles between vectors
- Light intensity (diffuse lighting)
- Projection of vectors
- Determining if surfaces face the camera

#### Cross Product

Produces a vector perpendicular to both input vectors:

```
A × B = (Ay*Bz - Az*By, Az*Bx - Ax*Bz, Ax*By - Ay*Bx)
```

```c
t_vec3 vec3_cross(t_vec3 a, t_vec3 b)
{
    return vec3_new(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}
```

**Applications:**
- Computing surface normals
- Building camera coordinate system
- Orientation calculations

#### Vector Length & Normalization

```c
// |V| = sqrt(Vx² + Vy² + Vz²)
double vec3_length(t_vec3 v)
{
    return sqrt(vec3_dot(v, v));
}

// V̂ = V / |V|
t_vec3 vec3_normalize(t_vec3 v)
{
    double len = vec3_length(v);
    if (len == 0)
        return v;
    return vec3_scale(v, 1.0 / len);
}
```

---

## 3. Ray Generation

### Camera Model

The camera uses a virtual viewport in front of it. For each pixel, we calculate the direction from the camera through that pixel.

```
Camera Parameters:
- position:     Camera location in world space
- direction:    Forward vector (where camera looks)
- right:        Right vector (perpendicular to direction)
- up:           Up vector (perpendicular to direction and right)
- fov:          Field of view in degrees
- aspect_ratio: width / height
```

### Ray Generation Formula

```c
t_ray ray_from_camera(t_camera *cam, int x, int y, t_img *img)
{
    double fov_scale = tan((cam->fov * M_PI / 180.0) / 2.0);

    // Convert pixel coordinates to normalized device coordinates [-1, 1]
    double u = (2.0 * ((double)x + 0.5) / (double)img->width - 1.0);
    double v = 1.0 - 2.0 * ((double)y + 0.5) / (double)img->height;

    // Apply aspect ratio and FOV
    u *= cam->aspect_ratio * fov_scale;
    v *= fov_scale;

    // Calculate ray direction
    t_vec3 dir = cam->direction
               + u * cam->right
               + v * cam->up;

    return ray_new(cam->position, normalize(dir));
}
```

### Camera Vectors Update

When the camera rotates, we need to recalculate its coordinate system:

```c
void camera_update_vectors(t_camera *cam)
{
    // Direction from yaw and pitch angles
    cam->direction.x = cos(cam->pitch) * sin(cam->yaw);
    cam->direction.y = sin(cam->pitch);
    cam->direction.z = -cos(cam->pitch) * cos(cam->yaw);

    // Right vector (perpendicular to direction and world up)
    t_vec3 world_up = vec3_new(0, 1, 0);
    cam->right = normalize(cross(cam->direction, world_up));

    // Up vector (perpendicular to direction and right)
    cam->up = cross(cam->right, cam->direction);
}
```

---

## 4. Ray-Object Intersections

### 4.1 Ray-Sphere Intersection

A sphere is defined by its center **C** and radius **r**:

```
|P - C|² = r²
```

Substituting the ray equation P(t) = O + tD:

```
|O + tD - C|² = r²
```

Let **oc** = O - C, this expands to a quadratic equation:

```
(D·D)t² + 2(oc·D)t + (oc·oc - r²) = 0

a = D · D = 1 (if D is normalized)
b = 2 * (oc · D)
c = oc · oc - r²

discriminant = b² - 4ac
```

**Cases:**
- `disc < 0`: No intersection (ray misses sphere)
- `disc = 0`: One intersection (ray tangent to sphere)
- `disc > 0`: Two intersections (ray passes through sphere)

```c
t_hit intersect_sphere(t_ray ray, t_sphere *sp)
{
    t_vec3 oc = vec3_sub(ray.origin, sp->center);
    double r = sp->diameter / 2.0;

    double a = vec3_dot(ray.direction, ray.direction);
    double b = 2.0 * vec3_dot(oc, ray.direction);
    double c = vec3_dot(oc, oc) - r * r;

    double disc = b * b - 4 * a * c;
    if (disc < 0)
        return hit_new(); // No hit

    // Take the closest positive intersection
    double t = (-b - sqrt(disc)) / (2.0 * a);
    if (t < EPSILON)
        t = (-b + sqrt(disc)) / (2.0 * a);
    if (t < EPSILON)
        return hit_new();

    // Calculate hit point and normal
    hit.point = ray_at(ray, t);
    hit.normal = normalize(hit.point - sp->center);
    return hit;
}
```

**Normal at hit point:**
```
N = normalize(P - C)
```

---

### 4.2 Ray-Plane Intersection

A plane is defined by a point **P₀** and a normal **N**:

```
(P - P₀) · N = 0
```

Substituting the ray equation:

```
(O + tD - P₀) · N = 0
t = (P₀ - O) · N / (D · N)
```

```c
t_hit intersect_plane(t_ray ray, t_plane *pl)
{
    double denom = vec3_dot(ray.direction, pl->normal);

    // Ray parallel to plane
    if (fabs(denom) < EPSILON)
        return hit_new();

    t_vec3 po = vec3_sub(pl->point, ray.origin);
    double t = vec3_dot(po, pl->normal) / denom;

    if (t < EPSILON)
        return hit_new();

    hit.point = ray_at(ray, t);
    hit.normal = pl->normal;

    // Flip normal if we hit from behind
    if (denom > 0)
        hit.normal = vec3_negate(hit.normal);

    return hit;
}
```

---

### 4.3 Ray-Cylinder Intersection

A cylinder is defined by:
- **C**: Base center
- **A**: Axis direction (normalized)
- **r**: Radius
- **h**: Height

#### Infinite Cylinder Equation

For a cylinder along axis A through point C:

```
|P - C - ((P - C) · A) * A|² = r²
```

This removes the component along the axis, leaving only the perpendicular distance.

#### Finite Cylinder

After finding intersection with the infinite cylinder, check if the hit point is within height bounds:

```
0 ≤ (P - C) · A ≤ h
```

```c
static t_hit intersect_cylinder_body(t_ray ray, t_cylinder *cy)
{
    t_vec3 oc = vec3_sub(ray.origin, cy->center);

    // Project ray direction and oc perpendicular to axis
    t_vec3 d_perp = ray.direction - (ray.direction · cy->axis) * cy->axis;
    t_vec3 oc_perp = oc - (oc · cy->axis) * cy->axis;

    // Quadratic coefficients
    double a = d_perp · d_perp;
    double b = 2.0 * (d_perp · oc_perp);
    double c = oc_perp · oc_perp - r²;

    // Solve quadratic, check height bounds...
}
```

#### Cylinder Caps

Caps are circular disks at each end. Intersect with the plane at each cap, then check if hit point is within radius:

```c
// Check bottom cap (at center)
cap_hit = intersect_plane(ray, cap_plane);
if (cap_hit.hit)
{
    dist = |cap_hit.point - cy->center|;
    if (dist <= radius)
        // Valid cap hit
}

// Check top cap (at center + height * axis)
```

---

### 4.4 Ray-Cone Intersection

A cone is defined by:
- **apex**: Tip point of the cone
- **A**: Axis direction (from base to tip)
- **θ**: Half-angle
- **h**: Height

#### Cone Equation

For a cone with apex at **T** and axis **A**:

```
(D · A)² - cos²(θ) * (D · D) = 0     // Direction component
```

The full equation leads to a quadratic in t:

```c
tip = apex + height * axis;  // Tip is at the top
cos2 = cos²(angle);
oc = ray.origin - tip;

dv = D · A;
ocv = oc · A;

a = dv² - cos² * (D · D)
b = 2 * (dv * ocv - cos² * (D · oc))
c = ocv² - cos² * (oc · oc)
```

#### Height Constraint

After finding t, verify the point is within cone bounds:

```
-height ≤ (P - tip) · A ≤ 0
```

#### Cone Normal

The normal at a point P on the cone surface:

```
N = normalize(P - tip - ((P - tip) · A / cos²) * A)
```

#### Cone Cap

The base is a circular disk at the apex. Check intersection with the base plane, then verify the point is within the base radius:

```
base_radius = height * tan(angle)
```

---

## 5. Lighting Model

### Phong Reflection Model

The total illumination at a point is:

```
I = Ia + Id + Is

I  = Total intensity
Ia = Ambient component
Id = Diffuse component
Is = Specular component
```

### 5.1 Ambient Light

Simulates indirect lighting that illuminates all surfaces equally:

```
Ia = ka * La * Cobject

ka = ambient coefficient (0-1)
La = ambient light color
Cobject = object color
```

```c
ambient = object_color * ambient_color * ambient_ratio;
```

### 5.2 Diffuse Reflection (Lambertian)

Models light scattered equally in all directions from a matte surface:

```
Id = kd * Ld * max(0, N · L) * Cobject

kd = diffuse coefficient (light brightness)
Ld = light color
N  = surface normal
L  = direction to light (normalized)
```

```c
t_vec3 calculate_diffuse(t_hit hit, t_light *light)
{
    t_vec3 light_dir = normalize(light->position - hit.point);

    double diff = max(0, dot(hit.normal, light_dir));

    return hit.color * light->color * diff * light->brightness;
}
```

### 5.3 Specular Reflection (Phong)

Models shiny highlights on reflective surfaces:

```
Is = ks * Ls * max(0, R · V)^n

ks = specular coefficient
Ls = light color
R  = reflection direction
V  = view direction (toward camera)
n  = shininess exponent (higher = smaller highlight)
```

**Reflection Vector:**
```
R = 2 * (N · L) * N - L
```

```c
t_vec3 calculate_specular(t_hit hit, t_light *light, t_vec3 view_dir)
{
    t_vec3 light_dir = normalize(light->position - hit.point);
    double dot_nl = dot(hit.normal, light_dir);

    if (dot_nl < 0)
        return vec3_new(0, 0, 0);

    // Reflection: R = 2(N·L)N - L
    t_vec3 reflect = 2.0 * dot_nl * hit.normal - light_dir;
    reflect = normalize(reflect);

    double spec = max(0, dot(reflect, view_dir));
    spec = pow(spec, SPECULAR_EXP) * SPECULAR_STRENGTH * light->brightness;

    return light->color * spec;
}
```

### Combined Lighting

```c
t_vec3 calculate_lighting(t_hit hit, t_scene *scene, t_vec3 view_dir)
{
    // Ambient
    t_vec3 result = hit.color * scene->ambient.color * scene->ambient.ratio;

    // For each light
    for (int i = 0; i < scene->light_count; i++)
    {
        if (!is_in_shadow(hit.point, light_dir, light_dist, scene))
        {
            result += calculate_diffuse(hit, &scene->lights[i]);
            result += calculate_specular(hit, &scene->lights[i], view_dir);
        }
    }

    return result;
}
```

---

## 6. Shadow Computation

### Shadow Rays

To determine if a point is in shadow:

1. Cast a ray from the hit point toward each light
2. If any object blocks the path, the point is in shadow for that light

```c
bool is_in_shadow(t_vec3 point, t_vec3 light_dir, double light_dist, t_scene *scene)
{
    // Offset origin slightly to avoid self-intersection
    t_vec3 origin = point + light_dir * EPSILON * 10;

    t_ray shadow_ray;
    shadow_ray.origin = origin;
    shadow_ray.direction = light_dir;

    t_hit hit = find_closest_hit(shadow_ray, scene);

    // In shadow if we hit something before reaching the light
    return (hit.hit && hit.t < light_dist);
}
```

### Shadow Acne Prevention

Without the EPSILON offset, shadow rays might intersect with the surface they originate from due to floating-point precision errors, causing "shadow acne" artifacts.

---

## 7. MiniLibX Graphics

### Initialization

```c
void *mlx = mlx_init();                           // Initialize MLX
void *win = mlx_new_window(mlx, width, height, "miniRT");  // Create window
```

### Image Buffer

Instead of drawing pixels one by one (slow), we use an image buffer:

```c
typedef struct s_img
{
    void    *img_ptr;      // MLX image pointer
    char    *pixels_ptr;   // Direct pixel data access
    int     bpp;           // Bits per pixel
    int     line_len;      // Bytes per line
    int     endian;        // Byte order
    int     width;
    int     height;
} t_img;

// Create image
img.img_ptr = mlx_new_image(mlx, width, height);
img.pixels_ptr = mlx_get_data_addr(img.img_ptr, &img.bpp, &img.line_len, &img.endian);
```

### Pixel Manipulation

```c
void put_pixel(t_img *img, int x, int y, int color)
{
    if (x < 0 || x >= img->width || y < 0 || y >= img->height)
        return;

    int offset = y * img->line_len + x * (img->bpp / 8);
    *(unsigned int *)(img->pixels_ptr + offset) = color;
}
```

### Color Encoding

Colors are stored as 32-bit integers (ARGB or BGRA depending on endianness):

```c
int vec3_to_color(t_vec3 color)
{
    int r = (int)(clamp(color.x, 0, 255));
    int g = (int)(clamp(color.y, 0, 255));
    int b = (int)(clamp(color.z, 0, 255));

    return (r << 16) | (g << 8) | b;
}
```

### Display & Event Loop

```c
mlx_put_image_to_window(mlx, win, img.img_ptr, 0, 0);  // Display image
mlx_hook(win, EVENT_TYPE, MASK, handler_function, param);  // Event handlers
mlx_loop(mlx);  // Start event loop
```

---

## 8. Data Structures

### Scene Structure

```c
typedef struct s_scene
{
    t_ambient   ambient;        // Ambient light
    t_camera    camera;         // View camera
    t_light     *lights;        // Array of lights
    int         light_count;
    t_object    *objects;       // Array of objects
    int         object_count;
} t_scene;
```

### Hit Record

Stores information about a ray-object intersection:

```c
typedef struct s_hit
{
    bool        hit;            // Did we hit something?
    double      t;              // Distance along ray
    t_vec3      point;          // Hit point in world space
    t_vec3      normal;         // Surface normal at hit point
    t_vec3      color;          // Object color
    double      specular;       // Specular coefficient
    int         checkerboard;   // Checkerboard flag
    double      u, v;           // Texture coordinates
    t_texture   *texture;       // Optional texture
    t_texture   *bump_map;      // Optional bump map
} t_hit;
```

### Object Union

```c
typedef struct s_object
{
    t_obj_type  type;           // OBJ_SPHERE, OBJ_PLANE, etc.
    t_sphere    sphere;
    t_plane     plane;
    t_cylinder  cylinder;
    t_cone      cone;
    int         checkerboard;
} t_object;
```

---

## Constants Reference

```c
#define EPSILON         0.0001      // Floating-point tolerance
#define CHECKER_SCALE   2.0         // Checkerboard size
#define SPECULAR_EXP    32.0        // Phong shininess
#define SPECULAR_STRENGTH 0.5       // Specular intensity
#define BUMP_STRENGTH   0.8         // Bump map intensity
```

---

## Performance Considerations

1. **Normalized Directions**: Pre-normalize ray directions to avoid repeated normalization
2. **Early Exit**: Return as soon as an intersection is found for shadow rays
3. **EPSILON Tolerance**: Use small epsilon values to handle floating-point precision
4. **Image Buffer**: Render to memory buffer, then display once (vs. per-pixel display)
5. **Dual Resolution**: Low resolution for real-time navigation, high resolution for final renders

