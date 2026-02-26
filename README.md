# kinect2pipe-IR
A small application for piping output from `libfreenect2` to a `v4l2loopback` device so the Kinect 2 can be used for unlocking the desktop with [Howdy](https://github.com/boltgolt/howdy/), like you can do with Windows Hello on Windows.

This is a fork of [kinect2pipe](https://github.com/swedishborgie/kinect2pipe), which does the same thing but with the RGB color stream, so you can use it as a normal webcam in the browser and other applications. This fork is focused on the IR stream, which is what Howdy uses for facial recognition.

If you want to use the RGB stream as well you can install the original, but read this guide first as it is more up to date, especially the prerequisites section.

This application uses the [libfreenect2](https://github.com/OpenKinect/libfreenect2) library to connect to the Kinect 2
and get IR frames (which are just 32-bit floating-point values) and then uses [libswscale](https://ffmpeg.org/libswscale.html) to perform a colorspace
conversion to YUV420P which is a much more generally supported format by linux apps.
Then, it will stream the frames to a virtual video device that basically any app can easily read by looking at `/dev/videoX` (where X is the number of the device, it will be 11 if you follow the instructions in this guide).
Since this program is quite CPU
intensive, it will only start the stream when client applications open a file handle to the
`v4l2loopback` device and will close the video stream when any handle is closed (this may be an issue if there are two consumers but usually it's good because it usually manages to shut itself down even if some system service or browser forgets to let go). It uses `inotify` to achieve this.

## Instructions

### Prerequisites

#### Installing dependencies

This varies by distribution. On Arch:
```bash
sudo pacman -S --needed base-devel cmake opencl-headers git glfw
```

#### Installing libfreenect2

1. Clone the libfreenect2 repository:
```bash
git clone https://github.com/OpenKinect/libfreenect2
cd libfreenect2
```

2. Patch the source code to allow compilation on newer software stacks. I don't really know why this works but it does.
```bash
sed -i 's/CL_ICDL_VERSION/MY_CL_ICDL_VERSION/g' src/opencl_depth_packet_processor.cpp
sed -i 's/CL_ICDL_VERSION/MY_CL_ICDL_VERSION/g' src/opencl_kde_depth_packet_processor.cpp
```

3. Compile and install the library:
```bash
mkdir -p build && cd build

# Run CMake (the explicit policy version is required to compile with newer CMake versions)
cmake -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_INSTALL_PREFIX=/usr ..

# Compile with all cores
make -j$(nproc)

# Install
sudo make install
```

#### Installing libswscale

This varies by distribution. On Arch, it should already be installed as a dependency of `ffmpeg`, check with:
```bash
pacman -Qs libswscale
```

### Configuring

1. **Skip this step if you already installed the [RGB version](https://github.com/swedishborgie/kinect2pipe)** Load the `video4l2loopback` kernel module at boot with the following command:
```bash
echo "v4l2loopback" | sudo tee /etc/modules-load.d/kinect.conf
```

2. Create the v4l2loopback virtual devices at boot with the following command. We are only setting `exclusive_caps=1 max_buffers=2` on `video10`, which are needed for browsers to see the device. This is because they open handles at boot and never release them, so the IR emitter would be on all the time. Change if needed by adding `,1` and `,2` after the `1` and `2` in the command:
```bash
# If you have the RGB version (this will put it on /dev/video10, change that number to what you had it before if needed). You can also change the number 11 or the card_label if you want:
sudo rm /etc/modprobe.d/v4l2loopback.conf
echo 'options v4l2loopback devices=2 video_nr=10,11 card_label=Kinect_RGB,Kinect_IR exclusive_caps=1 max_buffers=2' | sudo tee /etc/modprobe.d/kinect.conf

# If you don't have the RGB version. You can change the number 11 or the card_label if you want:
echo 'options v4l2loopback video_nr=11 card_label=Kinect_IR' | sudo tee /etc/modprobe.d/kinect_IR.conf
```

3. Reboot

### Compiling

1. Clone this repository:
```bash
git clone https://github.com/stignarnia/kinect2pipe-IR
cd kinect2pipe-IR
```

2. Compile and install the application. You can change the number 11 if you did it in the configuration step:
```bash
mkdir -p build && cd build
cmake -DLOOPBACK_DEVICE=/dev/video11 ..
sudo make install
```

### Usage

You can test everything is working using VLC (or any other V4L2 client, like the browser):

```bash
vlc v4l2:///dev/video11
```
    
You should get IR video output:
![example snapshot](snapshot.png)

#### Backup device (optional)

You can pass a second argument pointing to a regular V4L2 capture device. When the Kinect 2 is not present and a client asks for it, the application will serve frames from that device instead.

```bash
cmake -DLOOPBACK_DEVICE=/dev/video11 -DBACKUP_PATH=/dev/<yourdevice> ..
sudo make install
```

The backup device should output one of the following pixel formats (tried in preference order): YUYV, UYVY, YUV420, NV12, BGR24, RGB24. Frames are scaled to 512 × 424 with bilinear interpolation before being written to the loopback device.

`<yourdevice>` can be found by looking at the output of `v4l2-ctl --list-devices` or by looking at the symlinks in `/dev/v4l/by-id/` and `dev/v4l/by-path/` (recommended since they are usually more stable).

#### Hardware acceleration (optional)

The program defaults to having this off (never using the GPU). This is because the GPU isn't available in the pre login environment where this needs to run for face unlock. You can turn it on the same way as the backup device, by passing `-DHWACCEL=ON` to CMake. You will gain a more fluid video stream (useless for face unlock) but it will only work after you login the first time (for runtime things like `sudo`, `plymouth` and the `Win+L` lock screen but not at boot). You can also just run the application with that flag if you need it for other purposes that don't require a system service to run all the time:

```bash
./kinect2pipe_IR /dev/video11 --hwaccel
```

#### Configuring Howdy

```bash
sudo howdy config
```

Then change the `device_path` parameter to the correct video device (by default, `/dev/video11`).

Also, change `dark_threshold` to `90`.

```bash
sudo howdy add
```

Then follow the instructions to add a new face. You can add multiple faces if you want, or the same face with different expressions or lighting conditions.

```bash
sudo howdy test
```

You should see a live preview of the IR stream with a green box around your face if it is recognized and a red box if it is not.

This doesn't mean it will magically unlock your desktop yet, follow your distribution's instructions to enable Howdy for unlocking the desktop. On Arch, [this is the wiki](https://wiki.archlinux.org/title/Howdy).

#### Arch specific suggestions

Install Howdy from the AUR with:
```bash
yay -S howdy-git
# NOT just howdy or howdy-beta-git
```

When the wiki asks you to modify a file in `/etc/pam.d/`, just pick `system-auth` and it should apply to everything, but this is quite dangerous because if the configuration is wrong you can get locked out of your system, so be careful and make sure you have a backup plan to access your system if something goes wrong.