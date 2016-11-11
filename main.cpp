#include <iostream>
#include <vector>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

static const int src_img_rows = 700;
static const int src_img_cols = 800;

using namespace cv;
using namespace std;


Point2i calculate_center(Mat);
void getCoordinates(int event, int x, int y, int flags, void* param);
Mat undist(Mat);
double get_points_distance(Point2i, Point2i);
void set_target(cv::Point2i& targt, std::vector<cv::Point2i>& allTarget, cv::Mat& dst_img);
std::string move_direction(cv::Point2i Current, cv::Point2i Previous, cv::Point2i Target);
bool is_update_target(cv::Point2i Current, cv::Point2i Target);
void colorExtraction(cv::Mat* src, cv::Mat* dst,
	int code,
	int ch1Lower, int ch1Upper,
	int ch2Lower, int ch2Upper,
	int ch3Lower, int ch3Upper
	);

Mat image1;
Mat src_img, src_frame;
Mat element = Mat::ones(3, 3, CV_8UC1); //@comment �ǉ��@3�~3�̍s��ŗv�f�͂��ׂ�1�@dilate�����ɕK�v�ȍs��
int Ax, Ay, Bx, By, Cx, Cy, Dx, Dy;
int Tr, Tg, Tb;
Point2i pre_point; //@comment Point�\����<int�^>

int flag = 0;
//int ct = 0;
Mat dst_img, colorExtra;

ofstream ofs("out4.csv");
//@����o�͗p�ϐ�
const string  str = "test.avi";

Point2i target, P0 = { 0, 0 }, P1 = { 0, 0 };
std::vector<Point2i> allTarget;
std::vector<Point2i>::iterator target_itr;
string action;


int main(int argc, char *argv[])
{

	//@comment �J�����̌Ăяo�� pc�̃J���� : 0 web�J���� : 1 
	VideoCapture cap(1);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 640); //@comment web�J�����̉�����ݒ�
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480); //@comment web�J�����̏c����ݒ�
	if (!cap.isOpened()) return -1; //@comment �Ăяo���~�X������ΏI��

	VideoWriter write("out2.avi", CV_FOURCC('M', 'J', 'P', 'G'), cap.get(CV_CAP_PROP_FPS),
		cv::Size(src_frame.rows, src_frame.cols), true);
	if (!write.isOpened()) return -1;

	namedWindow("src", 1);
	namedWindow("dst", 1);
	namedWindow("video", 1);
	namedWindow("test1", 1);
	namedWindow("binari", 1);

	cap >> src_frame; //@comment 1�t���[���擾
	resize(src_frame, src_frame, Size(src_img_cols, src_img_rows), CV_8UC3); //@�擾�摜�̃��T�C�Y
	//src_img = undist(src_img) ; //@comment �J�����̘c�݂��Ƃ�(GoPro����)


	//------------------���W�擾-----------------------------------------------
	//@comment �摜������}�E�X��4�_���擾���̌�ESC�L�[�������ƕϊ��������J�n����

	namedWindow("getCoordinates");
	imshow("getCoordinates", src_frame);
	//@comment �ϊ��������l�p�`�̎l���̍��W���Ƃ�(�N���b�N)
	cvSetMouseCallback("getCoordinates", getCoordinates, NULL);
	waitKey(0);
	destroyAllWindows();


	//------------------�����ϊ�-----------------------------------------------
	Point2f pts1[] = { Point2f(Ax, Ay), Point2f(Bx, By),
		Point2f(Cx, Cy), Point2f(Dx, Dy) };

	Point2f pts2[] = { Point2f(0, src_img_rows), Point2f(0, 0),
		Point2f(src_img_cols, 0), Point2f(src_img_cols, src_img_rows) };

	//@comment �����ϊ��s����v�Z
	Mat perspective_matrix = getPerspectiveTransform(pts1, pts2);
	Mat dst_img, colorExtra;

	//@comment �ϊ�(���`�⊮)
	warpPerspective(src_frame, dst_img, perspective_matrix, src_frame.size(), INTER_LINEAR);

	//@comment �ϊ��O��̍��W��`��
	line(src_frame, pts1[0], pts1[1], Scalar(255, 0, 255), 2, CV_AA);
	line(src_frame, pts1[1], pts1[2], Scalar(255, 255, 0), 2, CV_AA);
	line(src_frame, pts1[2], pts1[3], Scalar(255, 255, 0), 2, CV_AA);
	line(src_frame, pts1[3], pts1[0], Scalar(255, 255, 0), 2, CV_AA);
	line(src_frame, pts2[0], pts2[1], Scalar(255, 0, 255), 2, CV_AA);
	line(src_frame, pts2[1], pts2[2], Scalar(255, 255, 0), 2, CV_AA);
	line(src_frame, pts2[2], pts2[3], Scalar(255, 255, 0), 2, CV_AA);
	line(src_frame, pts2[3], pts2[0], Scalar(255, 255, 0), 2, CV_AA);

	namedWindow("plotCoordinates", 1);
	imshow("plotCoordinates", src_frame);

	namedWindow("dst", 1);
	imshow("dst", dst_img);


	int frame = 0; //@comment �t���[�����ێ��ϐ�

	set_target(target, allTarget, dst_img);
	target_itr = allTarget.begin();

	while (1){

		if (target_itr == allTarget.end()){
			std::cout << "oooooooooooooooooooooooooooooooooooout" << std::endl;
			break;
		}

		cap >> src_frame;

		if (frame % 15 == 0){ //@comment�@�t���[���̎擾���𒲐߉\

			//@comment �摜�����T�C�Y(�傫������ƃf�B�X�v���C�ɓ����Ȃ�����)
			resize(src_frame, src_frame, Size(src_img_cols, src_img_rows), CV_8UC3);
			//src_frame = undist(src_frame); //@comment �J�����̘c�݂��Ƃ�(GoPro����)

			//--------------------�O���[�X�P�[����---------------------------------------

			//�ϊ�(���`�⊮)
			warpPerspective(src_frame, dst_img, perspective_matrix, src_frame.size(), INTER_LINEAR);
			//@comment hsv�𗘗p���ĐԐF�𒊏o
			//���͉摜�A�o�͉摜�A�ϊ��Ah�ŏ��l�Ah�ő�l�As�ŏ��l�As�ő�l�Av�ŏ��l�Av�ő�l
			colorExtraction(&dst_img, &colorExtra, CV_BGR2HSV, 150, 180, 70, 255, 70, 255);
			cvtColor(colorExtra, colorExtra, CV_BGR2GRAY);//@comment �O���[�X�P�[���ɕϊ�


			//�Q�l��
			//------------------�������l�ڑ��p--------------------------------------------
			Mat binari_2;

			//----------------------��l��-----------------------------------------------
			threshold(colorExtra, binari_2, 0, 255, THRESH_BINARY);
			dilate(binari_2, binari_2, element, Point(-1, -1), 3); //�c������3�� �Ō�̈����ŉ񐔂�ݒ�

			//---------------------�d�S�擾---------------------------------------------
			Point2i point = calculate_center(binari_2);//@comment moment�Ŕ��F�����̏d�S�����߂�
			//cout << "posion: " << point.x << " " << point.y << endl;//@comment �d�S�_�̕\��
			int ypos;
			if (point.x != 0){
				ypos = src_img_rows - (point.y + 6 * ((1000 / point.y) + 1));
				//cout << point.x << " " << ypos << endl; //@comment �ϊ��摜���ł̃��{�b�g�̍��W(�d�S)
				ofs << point.x << ", " << ypos << endl; //@comment �ϊ�
			}

			//---------------------���{�b�g�̓���擾------------------------------------
			P1.x = point.x;	P1.y = src_img_rows-ypos;
			if (target_itr != allTarget.end() &&P1.x != 0 && P1.y != 0 && P0.x != 0 && P0.y != 0){
				line(dst_img, P1, P0, Scalar(255, 0, 0), 2, CV_AA);
			if (is_update_target(P1, *target_itr)){
					// �^�[�Q�b�g�̍X�V
				std::cout << "UPDATE" << std::endl;
					target_itr++;
				}
			action = move_direction(P0, P1, *target_itr);
			std::cout << "target: " << target_itr->x << ", " << target_itr->y << "	position: " << P1.x << ", " << P1.y
				<< "	move: " << action << std::endl;
			}
			P0 = P1;
			
			//-------------------�d�S�_�̃v���b�g----------------------------------------- 
			if (!point.y == 0){ //@comment point.y == 0�̏ꍇ��exception���N����( 0���Z )
				//@comment �摜�C�~�̒��S���W�C���a�C�F(��)�C�������C���(-1, CV_AA�͓h��Ԃ�)
				circle(dst_img, Point(point.x, point.y + 6 * ((1000 / point.y) + 1)), 8, Scalar(0, 0, 0), -1, CV_AA);
				cv::putText(dst_img, action, cv::Point(point.x, point.y + 6 * ((1000 / point.y) + 1)), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 0), 1.0, CV_AA);
			}

			//------------------�^�[�Q�b�g�̃v���b�g--------------------------------------
			int n = 0;
			for (vector<Point2i>::iterator itr = allTarget.begin(); itr != allTarget.end(); itr++){
				cv::circle(dst_img, cv::Point(*itr), 48, cv::Scalar(255, 255, 0), 3, 4);
				cv::putText(dst_img, std::to_string(n), cv::Point(*itr), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 0.5, CV_AA);
				n++;
			}
			cv::circle(dst_img, cv::Point(*target_itr), 48, cv::Scalar(0, 0, 0), 3, 4);

			//---------------------�\������----------------------------------------------

			//�I�������l������Ō���
			//line(dst_img, Point(Ax, Ay), Point(Bx, By), Scalar(200, 0, 0), 3);
			//line(dst_img, Point(Bx, By), Point(Cx, Cy), Scalar(0, 200, 0), 3);
			//line(dst_img, Point(Cx, Cy), Point(Dx, Dy), Scalar(200, 0, 0), 3);
			//line(dst_img, Point(Dx, Dy), Point(Ax, Ay), Scalar(0, 200, 0), 3);

			//imshow("drawing", src_img);
			//---------------------�ϊ���̉摜��1m�l���̏��ڌ`��----------------------------

			//imshow("dst_img", dst_img);

			//Mat Eximg = dst_img;

			for (int i = 0; i <= src_img_cols; i += 100){
				for (int j = 0; j <= src_img_rows; j += 100){

					line(dst_img, Point(i, j), Point(i, src_img_cols), Scalar(200, 200, 200), 3);
					line(dst_img, Point(i, j), Point(src_img_rows, j), Scalar(200, 200, 200), 3);
				}
			}

			//�����̈�ƊO���̈�̍쐬
			line(dst_img, Point(100, 100), Point(100, src_img_cols), Scalar(200, 0, 0), 2);
			line(dst_img, Point(100, 100), Point(src_img_rows, 100), Scalar(200, 0, 0), 2);
			line(dst_img, Point(src_img_rows, 100), Point(src_img_rows, src_img_cols), Scalar(200, 0, 0), 2);
			line(dst_img, Point(100, src_img_cols - 100), Point(src_img_rows, src_img_cols - 100), Scalar(200, 0, 0), 2);

			//imshow("video", src_frame);
			imshow("red_point", dst_img);//@comment �o�͉摜
			imshow("colorExt", colorExtra);//@comment �Ԓ��o�摜
			//cout << "frame" << ct++ << endl; //@comment frame���\��
			//write << dst_img;
			std::cout << frame << std::endl;
			if (src_frame.empty() || waitKey(30) >= 0)
			{
				destroyAllWindows();
				return 0;
			}
		}
		frame++;
		write << dst_img;
	}
	ofs.close(); //@comment �t�@�C���X�g���[���̉��

	while (1){}
}

//@comment 2�_�Ԃ̋����擾�֐�
double get_points_distance(Point2i point, Point2i pre_point){

	return sqrt((point.x - pre_point.x) * (point.x - pre_point.x)
		+ (point.y - pre_point.y) * (point.y - pre_point.y));
}

//@comment �d�S�擾�p�֐�
Point2i calculate_center(Mat gray)
{

	Point2i center = Point2i(0, 0);
	//std::cout << center << std::endl;
	Moments moment = moments(gray, true);

	if (moment.m00 != 0)
	{
		center.x = (int)(moment.m10 / moment.m00);
		center.y = (int)(moment.m01 / moment.m00);
	}

	return center;
}

//@comment ���͉摜����4�_��ݒ肷��֐�
void getCoordinates(int event, int x, int y, int flags, void* param)
{

	static int count = 0;
	switch (event){
	case CV_EVENT_LBUTTONDOWN://@comment ���N���b�N�������ꂽ��

		if (count == 0){
			Ax = x, Ay = y;
			cout << "Ax :" << x << ", Ay: " << y << endl;
		}
		else if (count == 1){
			Bx = x, By = y;
			cout << "Bx :" << x << ", By: " << y << endl;
		}
		else if (count == 2){
			Cx = x, Cy = y;
			cout << "Cx :" << x << ", Cy: " << y << endl;
		}
		else if (count == 3){
			Dx = x, Dy = y;
			cout << "Dx :" << x << ", Dy: " << y << endl;
		}
		else{
			cout << "rgb(" << x << "," << y << ")  ";

			Vec3b target_color = src_img.at<Vec3b>(y, x);
			uchar r, g, b;
			Tr = target_color[2];
			Tg = target_color[1];
			Tb = target_color[0];
			cout << "r:" << Tr << " g:" << Tg << " b:" << Tb << endl;
		}
		count++;
		break;
	default:
		break;
	}
}
//@comment �J�����L�����u���[�V�����p�֐�(gopro�p)
Mat undist(Mat src_img)
{
	Mat dst_img;

	//@comment �J�����}�g���b�N�X(gopro)
	Mat cameraMatrix = (Mat_<double>(3, 3) << 469.96, 0, 400, 0, 467.68, 300, 0, 0, 1);
	//@comment �c�ݍs��(gopro)
	Mat distcoeffs = (Mat_<double>(1, 5) << -0.18957, 0.037319, 0, 0, -0.00337);

	undistort(src_img, dst_img, cameraMatrix, distcoeffs);
	return dst_img;
}

//@comment �F���o�p�֐� 
void colorExtraction(cv::Mat* src, cv::Mat* dst,
	int code,
	int ch1Lower, int ch1Upper, //@comment H(�F��)�@�ŏ��A�ő�
	int ch2Lower, int ch2Upper, //@comment S(�ʓx)�@�ŏ��A�ő�
	int ch3Lower, int ch3Upper  //@comment V(���x)�@�ŏ��A�ő�
	)
{
	cv::Mat colorImage;
	int lower[3];
	int upper[3];

	cv::Mat lut = cv::Mat(256, 1, CV_8UC3);

	cv::cvtColor(*src, colorImage, code);

	lower[0] = ch1Lower;
	lower[1] = ch2Lower;
	lower[2] = ch3Lower;

	upper[0] = ch1Upper;
	upper[1] = ch2Upper;
	upper[2] = ch3Upper;

	for (int i = 0; i < 256; i++){
		for (int k = 0; k < 3; k++){
			if (lower[k] <= upper[k]){
				if ((lower[k] <= i) && (i <= upper[k])){
					lut.data[i*lut.step + k] = 255;
				}
				else{
					lut.data[i*lut.step + k] = 0;
				}
			}
			else{
				if ((i <= upper[k]) || (lower[k] <= i)){
					lut.data[i*lut.step + k] = 255;
				}
				else{
					lut.data[i*lut.step + k] = 0;
				}
			}
		}
	}
	//@comment LUT���g�p���ē�l��
	cv::LUT(colorImage, lut, colorImage);

	//@comment Channel���ɕ���
	std::vector<cv::Mat> planes;
	cv::split(colorImage, planes);

	//@comment �}�X�N���쐬
	cv::Mat maskImage;
	cv::bitwise_and(planes[0], planes[1], maskImage);
	cv::bitwise_and(maskImage, planes[2], maskImage);

	//@comemnt �o��
	cv::Mat maskedImage;
	src->copyTo(maskedImage, maskImage);
	*dst = maskedImage;

}

//@comment �^�[�Q�b�g�ݒ�
void set_target(cv::Point2i& targt, std::vector<cv::Point2i>& allTarget, cv::Mat& dst_img){

	int width = dst_img.rows;
	int height = dst_img.cols;
	allTarget.clear();

	int n = 0;
	for (int j = 0; j < (width / 100) - 1; j++){
		if (j % 2 == 0){
			for (int i = 0; i < (height / 100) - 2; i++){
				target.x = (i + 1) * width / (width / 100) + 50;	target.y = 800 - (j + 1) * height / (height / 100) - 50;
				allTarget.push_back(target);
				n++;
			}
		}
		else{
			for (int i = (height / 100) - 3; i >= 0; i--){
				target.x = (i + 1) * width / (width / 100) + 50;	target.y = 800 - (j + 1) * height / (height / 100) - 50;
				allTarget.push_back(target);
				n++;
			}
		}
	}
}

//@comment ���{�b�g�̓��쌈��֐�
std::string move_direction(cv::Point2i Current, cv::Point2i Previous, cv::Point2i Target){
	cv::Point2i P0 = Current - Previous;
	cv::Point2i P1 = Target - Current;

	double angle = asin(P0.cross(P1) / (sqrt(P0.x * P0.x + P0.y * P0.y) * sqrt(P1.x * P1.x + P1.y * P1.y))) / CV_PI * 180;
	if (P0.dot(P1) >= 0){
		if (P0.cross(P1) >= 0){
			angle = 360.0 - angle;
		}
		else if (P0.cross(P1) < 0){
			angle = -360.0 - angle;
		}
	}

	if (-30 < angle && angle < 30){
		return std::string("f");
	}
	else if (angle <= -30){
		return std::string("r");
	}
	else{
		return std::string("l");
	}
}

//@comment true->�^�[�Q�b�g�X�V, false->�^�[�Q�b�g�X�V�Ȃ�
bool is_update_target(cv::Point2i Current, cv::Point2i Target){
	//Target.y = src_img_rows - Target.y;
	int dx = Current.x - Target.x;
	int dy = Current.y - Target.y;
	double d = sqrt(dx * dx + dy * dy);

	std::cout << "test " << d << endl;

	if (d < 50.0){
		std::cout << "TRUEEEEEEEEEEEEEEEEEEEE" << std::endl;
		return true;
	}
	else return false;
}
