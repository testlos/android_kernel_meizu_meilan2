Kernel repository for Meizu M2 Mini (meilan2)
===========================
(based on https://github.com/visi0nary/android_kernel_elephone_p8000)

Getting Started
---------------

Clone a repository and checkout current active branch:

    git clone https://github.com/divis1969/android_kernel_meilan2.git
    cd android_kernel_meilan2
    git checkout origin/3.10

Build the code:

    export ARCH=arm64
    export CROSS_COMPILE=/path/to/your/toolchain/aarch64/aarch64-linux-android-x.x/bin/aarch64-linux-android-
    make -j 4 2>&1 | tee build.log

Kernel image with DTB:

    arch/arm64/boot/Image.gz-dtb

Current state
-------------

- Phone boots with this kernel
- Display/Touch are functional (see Known issues)
- Wifi is functional

Known Issues
-------------

- Display is non-functional after phone wake up
- Central(touch) key is not working (no driver yet)