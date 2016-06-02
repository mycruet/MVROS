#include <test_nodelet/Setting.h>


Setting::Setting(void)
{
}


Setting::Setting(int x, int y, int de_x, int de_y,CameraCalibratorType type):bszie_x_(x),bszie_y_(y),ctype_(type)
{

	deviation_x_ = 1.0;
	deviation_y_ = 1.0;
}

Setting::~Setting(void)
{
}
