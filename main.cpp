#include <iostream>
#include <vector>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <locale.h>


bool Ret;
HANDLE arduino;

/*
//���{�b�g�̓}�j���A�����[�h�̍ۂ̃p�P�b�g�𗘗p�����s������B
//���̓R�}���h
//0�`5:�I�[�g���[�h���̑��s�R�}���h
//8:�X�^���o�C���[�h
//9:�}�j���A�����[�h
*/

//���M���Xbee�̃A�h���X
byte const robotAddr[] = { byte(0x00), byte(0x13), byte(0xA2), byte(0x00), byte(0x40), byte(0x9E), byte(0xAE), byte(0xF7) };
//�e�����o�C�g
byte const A = byte(0x41), B = byte(0x42), C = byte(0x43), D = byte(0x44), E = byte(0x45), F = byte(0x46),
G = byte(0x47), H = byte(0x48), I = byte(0x49), J = byte(0x4a), K = byte(0x4b), L = byte(0x4c),
M = byte(0x4d), N = byte(0x4e), O = byte(0x4f), P = byte(0x50), Q = byte(0x51), R = byte(0x52),
S = byte(0x53), T = byte(0x54), U = byte(0x55), V = byte(0x56), W = byte(0x57), X = byte(0x58),
Y = byte(0x59), Z = byte(0x5a);

//���[�h���Ƃ�left��right��pwm
byte lPwm[] = { byte(0x00), byte(0x18), byte(0x18), byte(0x18), byte(0x08), byte(0x00), byte(0x10), byte(0x10), byte(0x0c), byte(0x0c) };
byte rPwm[] = { byte(0x00), byte(0x18), byte(0x08), byte(0x00), byte(0x18), byte(0x18), byte(0x0c), byte(0x08), byte(0x0c), byte(0x08) };


int src_img_cols = 0; //width
int src_img_rows = 0; //height


using namespace cv;
using namespace std;

Point2i calculate_center(Mat);
void getCoordinates(int event, int x, int y, int flags, void* param);
Mat undist(Mat);
double get_points_distance(Point2i, Point2i);
void set_target(cv::Point2i& targt, std::vector<cv::Point2i>& allTarget);
int robot_action(cv::Point2i Current, cv::Point2i Previous, cv::Point2i Target);
bool is_update_target(cv::Point2i Current, cv::Point2i Target);
std::string is_out(cv::Point2i Robot);
void colorExtraction(cv::Mat* src, cv::Mat* dst,
	int code,
	int ch1Lower, int ch1Upper,
	int ch2Lower, int ch2Upper,
	int ch3Lower, int ch3Upper
	);

void sentAigamoCommand(int command);
void sentManualCommand(byte command);

Mat image1;
Mat src_img, src_frame, test_image1, test_image2;
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

Point2i target, P0[5] = { { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 } }, P1 = { 0, 0 };
std::vector<Point2i> allTarget;
std::vector<Point2i>::iterator target_itr;
int action;
char action_str;
Point2i a, b, c, d;	//�����̈�̒��_
int width;
int depth;
int test_flag = 0;


int main(int argc, char *argv[])
{
	BYTE date = 1;

	//1.�|�[�g���I�[�v��
	arduino = CreateFile("COM4", GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	//2014/01/22�ǋL�@����łȂ���Ȃ��ꍇ�ɂ�"\\\\.\\COM7"�Ƃ���ƂȂ��邩������܂���B

	if (arduino == INVALID_HANDLE_VALUE) {
		printf("PORT COULD NOT OPEN\n");
		system("PAUSE");
		exit(0);
	}

	cout << "please enter width and depth of field (m)" << endl << "width : ";
	cin >> src_img_cols;
	cout << "depth : ";
	cin >> src_img_rows;

	if (src_img_cols < 300 || src_img_rows < 300)
	{
		cout << "please enter a number more than 3(m)" << endl;
		return -1;
	}

	//@comment �J�����̌Ăяo�� pc�̃J���� : 0 web�J���� : 1 
	VideoCapture cap(1);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 640); //@comment web�J�����̉�����ݒ�
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480); //@comment web�J�����̏c����ݒ�
	if (!cap.isOpened()) return -1; //@comment �Ăяo���~�X������ΏI��

	//VideoWriter write("out2.avi", CV_FOURCC('M', 'J', 'P', 'G'), cap.get(CV_CAP_PROP_FPS),
	//cv::Size(, src_img_cols), true);
	//if (!write.isOpened()) return -1;

	//namedWindow("src", 1);
	//namedWindow("dst", 1);
	//namedWindow("video", 1);
	//namedWindow("test1", 1);
	//namedWindow("binari", 1);

	//@comment �n�߂̕��̃t���[���͈Â��\��������̂œǂݔ�΂�
	for (int i = 0; i < 10; i++) {
		cap >> src_frame; //@comment 1�t���[���擾
	}

	resize(src_frame, test_image1, Size(src_img_cols, src_img_rows), CV_8UC3); //@�擾�摜�̃��T�C�Y
	//src_img = undist(src_img) ; //@comment �J�����̘c�݂��Ƃ�(GoPro����)
	imshow("test1", test_image1);

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
	warpPerspective(src_frame, dst_img, perspective_matrix, Size(src_img_cols, src_img_rows), INTER_LINEAR);

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

	imshow("dst", dst_img);

	int frame = 0; //@comment �t���[�����ێ��ϐ�
	Mat plot_img;
	dst_img.copyTo(plot_img);
	set_target(target, allTarget);
	target_itr = allTarget.begin();
	int color_r = 0, color_g = 0, color_b = 0;
	int color_flag = 0;



	//4.���M
	char id = A;
	char command = 's';
	int key;

	// �t�@�C����������
	ofs << "x��, y���i�␳�Ȃ��j, ypos�i�␳����j" << endl;

	while (1) {


		cap >> src_frame;
		if (frame % 1 == 0) { //@comment�@�t���[���̎擾���𒲐߉\
			///////
			//2.����M�o�b�t�@������
			Ret = SetupComm(arduino, 1024, 1024);
			if (!Ret) {
				printf("SET UP FAILED\n");
				CloseHandle(arduino);
				system("PAUSE");
				exit(0);
			}
			Ret = PurgeComm(arduino, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);
			if (!Ret) {
				printf("CLEAR FAILED\n");
				CloseHandle(arduino);
				exit(0);
			}

			//3.��{�ʐM�����̐ݒ�
			DCB dcb;
			GetCommState(arduino, &dcb);
			dcb.DCBlength = sizeof(DCB);
			dcb.BaudRate = 57600;
			dcb.fBinary = TRUE;
			dcb.ByteSize = 8;
			dcb.fParity = NOPARITY;
			dcb.StopBits = ONESTOPBIT;

			Ret = SetCommState(arduino, &dcb);
			if (!Ret) {
				printf("SetCommState FAILED\n");
				CloseHandle(arduino);
				system("PAUSE");
				exit(0);
			}


			//std::cin >> command;

			//�}�j���A�����[�h�ɕύX�R�}���h�̑��M
			//s:�X�^���o�C
			//m:�}�j���A��
			//a: �I�[�g

			key = waitKey(200);
			//cout << char(key) << endl;
			if (char(key) != -1){
				command = char(key);
				//cout << command << endl;


				if (command == 's') {
					sentManualCommand(byte(0x00));
					cout << command << endl;
				}
				if (command == 'm') {
					sentManualCommand(byte(0x01));
					cout << command << endl;
				}
				if (command == 'a') {
					sentManualCommand(byte(0x01));
					cout << command << endl;
				}
			}
			//�p�P�b�g�쐬�E���M
			//command:�V�[�P���X�ԍ�0�`5
			else if (command >= '0' && command <= '9') {
				sentAigamoCommand(int(command - '0'));
				//printf("left:%d, right:%d\n", lPwm[int(command - '0')], rPwm[int(command - '0')]);
			}
			//printf("next mode ->");




			//@comment �摜�����T�C�Y(�傫������ƃf�B�X�v���C�ɓ����Ȃ�����)
			resize(src_frame, test_image2, Size(src_img_cols, src_img_rows), CV_8UC3);
			//src_frame = undist(src_frame); //@comment �J�����̘c�݂��Ƃ�(GoPro����)

			//--------------------�O���[�X�P�[����---------------------------------------

			//�ϊ�(���`�⊮)
			warpPerspective(src_frame, dst_img, perspective_matrix, Size(src_img_cols, src_img_rows), INTER_LINEAR);
			//@comment hsv�𗘗p���ĐԐF�𒊏o
			//���͉摜�A�o�͉摜�A�ϊ��Ah�ŏ��l�Ah�ő�l�As�ŏ��l�As�ő�l�Av�ŏ��l�Av�ő�l h:(0-180)���ۂ�1/2
			colorExtraction(&dst_img, &colorExtra, CV_BGR2HSV, 160, 180, 70, 255, 70, 255);
			//colorExtraction(&dst_img, &colorExtra, CV_BGR2HSV, 145, 165,70, 255, 70, 255);
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
			int ydef = 0;
			if (point.x != 0) {
				ypos = src_img_rows - (point.y + 6 * ((1000 / point.y) + 1));
				ydef = src_img_rows - point.y;//@comment �␳�Ȃ����d�S
				cout << point.x << " " << ypos << endl; //@comment �ϊ��摜���ł̃��{�b�g�̍��W(�d�S)
				ofs << point.x << ", " << ydef << ", " << ypos << endl; //@comment �ϊ�
			}

			//---------------------���{�b�g�̓���擾------------------------------------
			//if (frame % 2 == 0){
				P1 = { point.x, src_img_rows - ypos };
				if (target_itr != allTarget.end() && P1.x != 0 && P1.y != 0 && point.x != 0 && point.y != 0) {
					line(dst_img, P1, P0[4], Scalar(255, 0, 0), 2, CV_AA);
					if (is_update_target(P1, *target_itr)) {
						// �^�[�Q�b�g�̍X�V
						//std::cout << "UPDATE" << std::endl;
						target_itr++;
						if (target_itr == allTarget.end()) {//@comment �C�e���[�^���Ō�܂ōs������ŏ��ɖ߂�
							target_itr = allTarget.begin();
						}
					}
					action = robot_action(P0[4], P1, *target_itr);
					//std::cout << "target: " << target_itr->x << ", " << target_itr->y << "	position: " << P1.x << ", " << P1.y
					//<< is_out(P1) << action << std::endl;

					for (int i = 1; i < 5; i++){
						P0[i] = P0[i - 1];
					}
				}
				else{
					action = 0;
				}
				P0[0] = P1;
			//}

			if (command == 'a'){
				sentAigamoCommand(action);
			}
			std::cout << "cmd " << int(command) << std::endl;

			//-------------------�d�S�_�̃v���b�g----------------------------------------- 
			if (!point.y == 0) { //@comment point.y == 0�̏ꍇ��exception���N����( 0���Z )
				//@comment �摜�C�~�̒��S���W�C���a�C�F(��)�C�������C���(-1, CV_AA�͓h��Ԃ�)
				circle(dst_img, Point(point.x, point.y), 8, Scalar(255, 255, 255), -1, CV_AA);
				circle(dst_img, Point(point.x, point.y + 6 * ((1000 / point.y) + 1)), 8, Scalar(0, 0, 0), -1, CV_AA);
				//@comment �d�S�_�̈ړ�����
				circle(plot_img, Point(point.x, point.y + 6 * ((1000 / point.y) + 1)), 8, Scalar(0, 0, 255), -1, CV_AA);
				if (waitKey(30) == 114) {
					namedWindow("plot_img", 1);
					imshow("plot_img", plot_img);
				}
			}

			//------------------�^�[�Q�b�g�̃v���b�g--------------------------------------
			int n = 0;
			for (vector<Point2i>::iterator itr = allTarget.begin(); itr != allTarget.end(); itr++) {
				cv::circle(dst_img, cv::Point(*itr), 48, cv::Scalar(255, 255, 0), 3, 4);
				cv::putText(dst_img, std::to_string(n), cv::Point(*itr), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 0.5, CV_AA);
				n++;
			}
			cv::circle(dst_img, cv::Point(*target_itr), 48, cv::Scalar(0, 0, 0), 3, 4);
			cv::putText(dst_img, is_out(P1), cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 1.0, CV_AA);
			switch (action){
			case 1: cv::putText(dst_img, "f", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 1.0, CV_AA); break;
			case 2: cv::putText(dst_img, "r (large)", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 1.0, CV_AA); break;
			case 3: cv::putText(dst_img, "r (small)", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 1.0, CV_AA); break;
			case 4: cv::putText(dst_img, "l (large)", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 1.0, CV_AA); break;
			case 5: cv::putText(dst_img, "l (small)", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 1.0, CV_AA); break;
			default: break;
			}

			//------------------�}�X�̃v���b�g--------------------------------------

			for (int i = 0; i <= src_img_cols; i += 100) {
				for (int j = 0; j <= src_img_rows; j += 100) {

					line(dst_img, Point(i, j), Point(i, src_img_cols), Scalar(200, 200, 200), 3);
					line(dst_img, Point(i, j), Point(src_img_rows, j), Scalar(200, 200, 200), 3);
				}
			}

			//------------------���i�̈�̃v���b�g--------------------------------------
			cv::Point2i A = { 100, src_img_rows - 100 }, B = { 100, 100 }, C = { src_img_cols - 100, 100 }, D = { src_img_cols - 100, src_img_rows - 100 };

			line(dst_img, Point(A), Point(B), Scalar(200, 0, 0), 3);
			line(dst_img, Point(B), Point(C), Scalar(200, 0, 0), 3);
			line(dst_img, Point(C), Point(D), Scalar(200, 0, 0), 3);
			line(dst_img, Point(D), Point(A), Scalar(200, 0, 0), 3);

			//---------------------�\������----------------------------------------------

			resize(dst_img, dst_img, Size(700, 700));
			resize(colorExtra, colorExtra, Size(600, 600));
			imshow("dst_image", dst_img);//@comment �o�͉摜
			imshow("colorExt", colorExtra);//@comment �Ԓ��o�摜
			imshow("plot_img", plot_img);
			//cout << "frame" << ct++ << endl; //@comment frame���\��
			//write << dst_img;
			//std::cout << frame << std::endl;

			//@comment "q"����������v���O�����I��
			if (src_frame.empty() || waitKey(30) == 113)
			{
				destroyAllWindows();
				return 0;
			}
		}
		frame++;
		//	write << dst_img;
	}

	ofs.close(); //@comment �t�@�C���X�g���[���̉��
	CloseHandle(arduino);
	system("PAUSE");
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
	switch (event) {
	case CV_EVENT_LBUTTONDOWN://@comment ���N���b�N�������ꂽ��

		if (count == 0) {
			Ax = x, Ay = y;
			cout << "Ax :" << x << ", Ay: " << y << endl;
		}
		else if (count == 1) {
			Bx = x, By = y;
			cout << "Bx :" << x << ", By: " << y << endl;
		}
		else if (count == 2) {
			Cx = x, Cy = y;
			cout << "Cx :" << x << ", Cy: " << y << endl;
		}
		else if (count == 3) {
			Dx = x, Dy = y;
			cout << "Dx :" << x << ", Dy: " << y << endl;
		}
		else {
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

	for (int i = 0; i < 256; i++) {
		for (int k = 0; k < 3; k++) {
			if (lower[k] <= upper[k]) {
				if ((lower[k] <= i) && (i <= upper[k])) {
					lut.data[i*lut.step + k] = 255;
				}
				else {
					lut.data[i*lut.step + k] = 0;
				}
			}
			else {
				if ((i <= upper[k]) || (lower[k] <= i)) {
					lut.data[i*lut.step + k] = 255;
				}
				else {
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
void set_target(cv::Point2i& targt, std::vector<cv::Point2i>& allTarget) {

	int height = src_img_rows;
	int width = src_img_cols;
	allTarget.clear();

	int n = 0;
	for (int j = 0; j < (height / 100) - 2; j++) {
		if (j % 2 == 0) {
			for (int i = 0; i < (width / 100) - 2; i++) {
				target.x = (i + 1) * width / (width / 100) + 50;	target.y = height - (j + 1) * height / (height / 100) - 50;
				allTarget.push_back(target);
				n++;
			}
		}
		else {
			for (int i = (width / 100) - 3; i >= 0; i--) {
				target.x = (i + 1) * width / (width / 100) + 50;	target.y = height - (j + 1) * height / (height / 100) - 50;
				allTarget.push_back(target);
				n++;
			}
		}
	}
}

//@comment ���{�b�g�̓��쌈��֐�
int robot_action(cv::Point2i Current, cv::Point2i Previous, cv::Point2i Target) {
	cv::Point2i P0 = Current - Previous;
	cv::Point2i P1 = Target - Current;

	int dx1 = abs(Current.x - 100);
	int dx2 = abs(Current.x - src_img_rows - 100);

	double angle = asin(P0.cross(P1) / (sqrt(P0.x * P0.x + P0.y * P0.y) * sqrt(P1.x * P1.x + P1.y * P1.y))) / CV_PI * 180;
	if (P0.dot(P1) >= 0) {
		if (P0.cross(P1) >= 0) {
			angle = 360.0 - angle;
		}
		else if (P0.cross(P1) < 0) {
			angle = -360.0 - angle;
		}
	}

	if (-30 < angle && angle < 30) {
		return 1;
	}
	else if (angle <= -30) {
		if ((dx1 < 50.0) || (dx2 < 50.0)) {
			return 2;
		}
		else return 3;
	}
	else {
		if ((dx1 < 50.0) || (dx2 < 50.0)) {
			return 4;
		}
		else return 5;
	}
}

//@comment true->�^�[�Q�b�g�X�V, false->�^�[�Q�b�g�X�V�Ȃ�
bool is_update_target(cv::Point2i Current, cv::Point2i Target) {
	//Target.y = src_img_rows - Target.y;
	int dx = Current.x - Target.x;
	int dy = Current.y - Target.y;
	double d = sqrt(dx * dx + dy * dy);

	if (d < 50.0) {
		return true;
	}
	else return false;
}


//@comment ���O����֐�
std::string is_out(cv::Point2i Robot) {
	cv::Point2i A = { 100, src_img_rows - 100 }, B = { 100, 100 }, C = { src_img_cols - 100, 100 }, D = { src_img_cols - 100, src_img_rows - 100 };
	cv::Point2i BA = A - B, BC = C - B, BP = Robot - B;
	cv::Point2i DC = C - D, DA = A - D, DP = Robot - D;
	int c1, c2, c3, c4;
	bool flag1 = false, flag2 = false;

	line(dst_img, Point(A), Point(B), Scalar(200, 200, 200), 3);
	line(dst_img, Point(B), Point(C), Scalar(200, 200, 200), 3);
	line(dst_img, Point(C), Point(D), Scalar(200, 200, 200), 3);
	line(dst_img, Point(D), Point(A), Scalar(200, 200, 200), 3);

	c1 = BA.cross(BP);
	c2 = BP.cross(BC);
	c3 = DC.cross(DP);
	c4 = DP.cross(DA);

	if ((c1 >= 0 && c2 >= 0) || (c1 < 0 && c2 < 0)) {
		flag1 = true;
	}
	if ((c3 >= 0 && c4 >= 0) || (c3 < 0 && c4 < 0)) {
		flag2 = true;
	}
	if (flag1 && flag2) {
		return "IN";
	}
	else return "OUT";
}



//�p�P�b�g�쐬�E���M
//command:�V�[�P���X�ԍ�0�`5
void sentAigamoCommand(int command) {

	DWORD dwSendSize;
	DWORD dwErrorMask;
	byte checksum = 0;

	//�p�P�b�g����
	byte requestPacket[] = { byte(0x7E), byte(0x00), byte(0x1F), byte(0x10), byte(0x01),
		robotAddr[0], robotAddr[1], robotAddr[2], robotAddr[3],
		robotAddr[4], robotAddr[5], robotAddr[6], robotAddr[7],
		byte(0xFF), byte(0xFE), byte(0x00), byte(0x00), A, G, S,
		M, F, A, T, A, L, 1, lPwm[byte(command)], R, 1, rPwm[byte(command)], A, G, E, byte(0x00) };

	std::cout << command << std::endl;

	//�`�F�b�N�T���̌v�Z
	for (int i = 3; i < 34; i++) {
		checksum += requestPacket[i];
	}
	checksum = 0xFF - (checksum & 0x00FF);
	requestPacket[34] = byte(checksum);

	//�p�P�b�g�̑��M
	Ret = WriteFile(arduino, requestPacket, sizeof(requestPacket), &dwSendSize, NULL);

	if (!Ret) {
		printf("SEND FAILED\n");
		CloseHandle(arduino);
		system("PAUSE");
		exit(0);
	}

}



//�}�j���A�����[�h�ɕύX�R�}���h�̑��M
//8:�X�^���o�C
//9:�}�j���A��
void sentManualCommand(byte command) {

	DWORD dwSendSize;
	DWORD dwErrorMask;
	byte checksum = 0;

	//�p�P�b�g����
	byte requestPacket[] = { byte(0x7E), byte(0x00), byte(0x1A), byte(0x10), byte(0x01),
		robotAddr[0], robotAddr[1], robotAddr[2], robotAddr[3],
		robotAddr[4], robotAddr[5], robotAddr[6], robotAddr[7],
		byte(0xFF), byte(0xFE), byte(0x00), byte(0x00), A, G, S, C, F, A, T, A, command, A, G, E, byte(0x00) };

	//�`�F�b�N�T���̌v�Z
	for (int i = 3; i < 29; i++) {
		checksum += requestPacket[i];
	}
	checksum = 0xFF - (checksum & 0x00FF);
	requestPacket[29] = byte(checksum);

	//�p�P�b�g�̑��M
	Ret = WriteFile(arduino, requestPacket, sizeof(requestPacket), &dwSendSize, NULL);

	if (!Ret) {
		printf("SEND FAILED\n");
		CloseHandle(arduino);
		system("PAUSE");
		exit(0);
	}

}