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

echo_and_run cd "/home/zpj/ladar_and_camera_fusion/src/Kalibr/Schweizer-Messer/sm_python"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/zpj/ladar_and_camera_fusion/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/zpj/ladar_and_camera_fusion/install/lib/python2.7/dist-packages:/home/zpj/ladar_and_camera_fusion/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/zpj/ladar_and_camera_fusion/build" \
    "/usr/bin/python2" \
    "/home/zpj/ladar_and_camera_fusion/src/Kalibr/Schweizer-Messer/sm_python/setup.py" \
     \
    build --build-base "/home/zpj/ladar_and_camera_fusion/build/Kalibr/Schweizer-Messer/sm_python" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/zpj/ladar_and_camera_fusion/install" --install-scripts="/home/zpj/ladar_and_camera_fusion/install/bin"
