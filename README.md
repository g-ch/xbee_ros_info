# xbeecom
 the minimize xbee communication code
# usage
* 先创建工作空间`mkdir -p xbeecom/src`
* 进入src `cd xbeecom/src`
* 拷贝整个代码进来到src根目录
* 回退上级目录，编译 `cd .. catkin_make`
* `source devel/setup.bash(zsh)`,写入到.zshrc,注意要在mavros之后source，会冲突（原因未知）