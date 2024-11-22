#include <iostream>
#include <mutex>
#include <thread>
#include <string>

using namespace std;
mutex mtx;
void releaseHydrogen(int& Hnum) {
	lock_guard<mutex> lck(mtx);
	while (Hnum >= 4) {

		Hnum -= 4;
		cout << "HHHH";
	}
}

void releaseCarbon(int& Cnum) {
	lock_guard<mutex> lck(mtx);
	while (Cnum >= 1) {
		Cnum -= 1;
		cout << "C";

	}
}

int main() {
	int H = 0, C = 0;
	string CH4;
	cin >> CH4;
	for (int i = 0; i < CH4.size(); i++) {
		if (CH4[i] == 'C')
			C++;
		else if (CH4[i] == 'H')
			H++;
	}
	int a = H / 4 - C;
	if (a < 0) C = C + a;
	else H = 4*C;
	thread t1(releaseHydrogen, ref(H));
	thread t2(releaseCarbon, ref(C));
	t1.join();
	t2.join();
	return 0;
}