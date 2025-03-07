#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/zjy/YanBot/src/Cerebellum/grounding_sam_ros_pkg"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/zjy/YanBot/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/zjy/YanBot/install/lib/python3/dist-packages:/home/zjy/YanBot/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/zjy/YanBot/build" \
    "/usr/bin/python3" \
    "/home/zjy/YanBot/src/Cerebellum/grounding_sam_ros_pkg/setup.py" \
     \
    build --build-base "/home/zjy/YanBot/build/Cerebellum/grounding_sam_ros_pkg" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/zjy/YanBot/install" --install-scripts="/home/zjy/YanBot/install/bin"
