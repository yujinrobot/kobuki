#!/bin/bash
#AUTHOR: Younghun Ju <yhju@yujinrobot.comm>, <yhju83@gmail.com>

# Basic setup
if [ "$ROS_DISTRO" == "" ]; then
  ROS_DISTRO=groovy
fi

if [ "$SOURCE" == "" ]; then
  SOURCE=hydro-devel
fi

if [ "$TARGET" == "" ]; then
  TARGET=gh-pages
fi

if [ "$COMMIT_MESSAGE" == "" ]; then
  COMMIT_MESSAGE="Updated doxygen(automated update)."
fi

WORKDIR=/tmp/kobuki_driver_doxygen
while [ -d $WORKDIR ]; do
  WORKDIR=/tmp/kobuki_driver_doxygen_$RANDOM
done
mkdir -p $WORKDIR

echo "rosdistro: "$ROS_DISTRO
echo "source branch: "$SOURCE
echo "target branch: "$TARGET
echo "working directory: "$WORKDIR
echo "commit message: "$COMMIT_MESSAGE
echo
echo

# Check the rosdoc-lite
source /opt/ros/$ROS_DISTRO/setup.bash
if [ "`which rosdoc_lite`" == "" ]; then
  sudo apt-get update
  sudo apt-get install ros-$ROS_DISTRO-rosdoc-lite
fi

# Clone kobuki repository into temporal working directory
cd $WORKDIR
git clone https://github.com/yujinrobot/kobuki

# Checkout source branch
cd kobuki
if [ "git branch | grep $SOURCE" == "" ]; then
  git branch -t $SOURCE origin/$SOURCE
fi
git checkout $SOURCE
git clean -fdx

# Build doxygen document
cd kobuki_driver
make doxygen
mv ./doc/html ../../doxygen

# Checkout target branch
cd ../
if [ "git branch | grep $SOURCE" == "" ]; then
  git branch -t $TARGET origin/$TARGET
fi
git checkout $TARGET
git clean -fdx

# Replace doxygen directory with new version
rm ./doxygen -rf
mv ../doxygen ./

# Add, commit and push.
git add ./doxygen
git commit -m "$COMMIT_MESSAGE"
git push origin $TARGET

if [ $? -eq 0 ]; then
  echo
  echo 'done'
else
  echo
  echo 'Authentification of repository failed.';
  echo 'Run below command to retry.'
  echo '   $ cd '$WORKDIR'/kobui'
  echo '   $ git push origin '$TARGET
  exit -1
fi


#### Cleanup
###rm -rf $WORKDIR
# Temporarily disable it, because of
#  1) $WORKDIR is in /tmp directory, it will be removed on shutdown
#  2) To let user do something, when authentification of repository is failed.

echo 'done'
