# Multi-Vehicle Lidar Point Cloud Generator

This project simulator point cloud streams on multiple vehicles using
[Carla simulator](https://github.com/carla-simulator/carla).

Supported Carla version is 0.9.14.

## Usage

```bash
cargo run --release -- \
    --map Town01 \
    --n-cars 10 \
    --output-dir output
```

## License

This project is licensed under MIT license. Please read the [license
file](LICENSE.txt).
