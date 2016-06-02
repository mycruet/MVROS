
class Setting
{


public:

    enum CameraCalibratorType
	{
        Opencv3_Chessboard_File = 0,
        Opencv3_Chessboard_Camera
	};



	Setting(void);
	Setting(int x, int y, int de_x, int de_y, CameraCalibratorType type);
	~Setting(void);
//private:
	int bszie_x_;
	int bszie_y_;
	double deviation_x_;
	double deviation_y_;
	CameraCalibratorType ctype_;
};

