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

echo_and_run cd "/home/robotarm/Documents/solomon_ws/src/aruco_hand_eye"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/robotarm/Documents/solomon_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/robotarm/Documents/solomon_ws/install/lib/python2.7/dist-packages:/home/robotarm/Documents/solomon_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/robotarm/Documents/solomon_ws/build" \
    "/usr/bin/python2" \
    "/home/robotarm/Documents/solomon_ws/src/aruco_hand_eye/setup.py" \
     \
    build --build-base "/home/robotarm/Documents/solomon_ws/build/aruco_hand_eye" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/robotarm/Documents/solomon_ws/install" --install-scripts="/home/robotarm/Documents/solomon_ws/install/bin"
