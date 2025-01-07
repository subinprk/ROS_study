#include<iostream>
#include<iomanip>
#include<random>
#include<cmath>
// #include<algorithm>

using namespace std;
random_device rd;
mt19937 gen(rd());

class pointData {
private:
	float x = 0;
	float y = 0;
public:
	float getX() { return this->x; }
	float getY() { return this->y; }
	void setPoint(float x, float y) {
		this->x = x;
		this->y = y;
	}
};

class PointCloud {
private:
	pointData* Point;
	int PCsize;
public:
	PointCloud() {}
	PointCloud(int PCsize) {
		uniform_real_distribution<float> distrib_point(-5, 5);
		this->PCsize = PCsize;
		this->Point = new pointData[PCsize];
		for (int i = 0; i < PCsize; i++) {
			this->Point[i].setPoint(distrib_point(gen), distrib_point(gen));
		}
	}
	float getPointX(int idx) { return this->Point[idx].getX(); }
	float getPointY(int idx) { return this->Point[idx].getY(); }
	float getPointDist(int idx) { return sqrt(pow(this->Point[idx].getX(), 2) + pow(this->Point[idx].getY(), 2)); }
	int getPointSize() { return PCsize; }
	void setPoint(int idx, float x, float y) { this->Point[idx].setPoint(x, y); }
};


void sort_PointCloud(PointCloud& PC) { //�������� �����ϱ�. �������� ���۸��ؼ� ����
	if (PC.getPointSize() <= 1) return; // Check if the PointCloud is empty or has only one point
	for (size_t i = 0; i < PC.getPointSize() - 1; ++i){
		for (size_t j = 0; j < PC.getPointSize() - i - 1; ++j){
			if (PC.getPointDist(j) > PC.getPointDist(j + 1))
				{
					float tempX = PC.getPointX(j);
					float tempY = PC.getPointY(j);
					PC.setPoint(j, PC.getPointX(j + 1), PC.getPointY(j + 1));
					PC.setPoint(j + 1, tempX, tempY);
				}
		}
	}
}

std::vector<pair<int, int>> findUniqList(PointCloud &PC) {
	std::vector<pair<int, int>> UniqList;

	for (size_t i = 0; i < PC.getPointSize(); i++) {
		int x = static_cast<int>(round(PC.getPointX(i)));
		int y = static_cast<int>(round(PC.getPointY(i)));
		bool found = false;

		for (const auto& point : UniqList) {
			if (point.first == x && point.second == y) {
				found = true;
				break;
			}
		}
		if (!found && (x < 5 && x > -5 && y < 5 && y > -5)) {
			UniqList.push_back(make_pair(x, y));
		}
	}
	return UniqList;
}

PointCloud DownSampling(PointCloud &PC) {

	int outputIdx = 0;
	std::vector<pair<int, int>> UniqList;
	std::vector<pair<int, int>>::iterator iter;

	pcList = findUniqList(PC);
	
	PointCloud outputPC(pcList.size());
	for (size_t i = 0; i < pcList.size(); ++i) {
		outputPC.setPoint(i, static_cast<float>(pcList[i].first), static_cast<float>(pcList[i].second));
	}
	return outputPC;
}

void printPointCloud(PointCloud PC) {
	cout << setprecision(3) << fixed;
	for (int i = 0; i < PC.getPointSize(); i++) {
		cout << "Point[" << i << "]\t\b\b\b\b\b -> X : " << PC.getPointX(i) <<
			"\t\b\b, Y : " << PC.getPointY(i) << "\t, Dist : " << PC.getPointDist(i) << endl;
	}
}

int main() {
	//generate random pointCloud
	uniform_int_distribution<int> distrib_sz(50, 100);
	PointCloud inputPC(distrib_sz(gen));
	PointCloud outputPC1;
    // PointCloud outputPC2;



	cout << " ------------ Input Point Cloud ------------ " << endl;
	printPointCloud(inputPC);		        //print all pointCloud

    // outputPC2 = ROI(inputPC);
    // cout << "------------ROI point Cloud--------------" << std::endl;
    // printPointCloud(outputPC2);

	sort_PointCloud(inputPC);               //apply sorting, ��������
	cout << " \n\n------------ Sorted Input Point Cloud ------------ " << endl;
	printPointCloud(inputPC);				//print all pointCloud

	outputPC1 = DownSampling(inputPC);		//apply DownSampling
	sort_PointCloud(outputPC1);              //apply sorting, ��������

	cout << " \n\n------------ Sorted output Point Cloud ------------ " << endl;
	printPointCloud(outputPC1);				//print all pointCloud

	return 0;
}